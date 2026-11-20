#include "source_trainer.h"

#include <algorithm>
#include <cstdio>

namespace dp {

namespace {
std::string hex_addr(uint32_t addr) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%05X", addr);
    return buf;
}

constexpr int kMaxCrLoops = 10;           // hard cap on CR iterations per attempt
constexpr int kMaxSameSwingTries = 5;     // classic rule: 5 tries at the same swing
constexpr int kMaxEqLoops = 5;
}  // namespace

// ---------------------------------------------------------------- AUX helpers

bool SourceTrainer::aux_read(uint32_t addr, uint8_t len, std::vector<uint8_t>& out) {
    AuxRequest req;
    req.cmd = AuxCmd::NativeRead;
    req.addr = addr;
    req.len = len;
    for (int attempt = 0; attempt <= cfg_.aux_max_retries; ++attempt) {
        AuxResponse rsp = aux_.transact(req);
        if (rsp.reply == AuxReply::Ack) {
            out = rsp.data;
            return true;
        }
        if (rsp.reply == AuxReply::Nack) break;   // NACK is final
        // DEFER or TIMEOUT: retry
    }
    last_error_ = "AUX read failed at " + hex_addr(addr);
    return false;
}

bool SourceTrainer::aux_write(uint32_t addr, const std::vector<uint8_t>& data) {
    AuxRequest req;
    req.cmd = AuxCmd::NativeWrite;
    req.addr = addr;
    req.data = data;
    req.len = static_cast<uint8_t>(data.size());
    for (int attempt = 0; attempt <= cfg_.aux_max_retries; ++attempt) {
        AuxResponse rsp = aux_.transact(req);
        if (rsp.reply == AuxReply::Ack) return true;
        if (rsp.reply == AuxReply::Nack) break;
    }
    last_error_ = "AUX write failed at " + hex_addr(addr);
    return false;
}

// ---------------------------------------------------------------- top level

TrainResult SourceTrainer::train() {
    TrainResult result;

    // 1. Read receiver capabilities.
    std::vector<uint8_t> caps;
    if (!aux_read(dpcd::DPCD_REV, dpcd::CAPS_SIZE, caps)) {
        result.fail_reason = last_error_;
        return result;
    }
    const uint8_t sink_rate_code = caps[dpcd::MAX_LINK_RATE];
    const uint8_t sink_lanes = caps[dpcd::MAX_LANE_COUNT] & 0x1F;
    sink_tps3_ = caps[dpcd::MAX_LANE_COUNT] & 0x40;
    sink_tps4_ = caps[dpcd::MAX_DOWNSPREAD] & 0x80;
    aux_rd_interval_ = caps[dpcd::TRAINING_AUX_RD_INTERVAL];

    const LinkRate start_rate =
        is_valid_rate_code(sink_rate_code)
            ? static_cast<LinkRate>(std::min(sink_rate_code, static_cast<uint8_t>(cfg_.max_rate)))
            : LinkRate::RBR;
    uint8_t lanes = std::min(sink_lanes, cfg_.max_lanes);
    if (lanes >= 4) lanes = 4; else if (lanes >= 2) lanes = 2; else lanes = 1;

    // 2. Try configurations: highest rate first; at RBR failure, halve the lane count.
    for (; lanes >= 1; lanes /= 2) {
        LinkRate rate = start_rate;
        while (true) {
            ++result.attempts;
            // LINK_BW_SET and LANE_COUNT_SET (bit 7 = enhanced framing) in one write.
            if (!aux_write(dpcd::LINK_BW_SET,
                           {static_cast<uint8_t>(rate), static_cast<uint8_t>(lanes | 0x80)})) {
                result.fail_reason = last_error_;
                return result;
            }

            Phase p = clock_recovery(lanes);
            if (p == Phase::Ok) p = channel_equalization(lanes, rate);

            if (p == Phase::AuxError) {
                aux_write(dpcd::TRAINING_PATTERN_SET, {0x00});
                result.fail_reason = last_error_;
                return result;
            }
            if (p == Phase::Ok) {
                // 3. Done: stop training, enable scrambling, start sending idle/video.
                if (!aux_write(dpcd::TRAINING_PATTERN_SET, {0x00})) {
                    result.fail_reason = last_error_;
                    return result;
                }
                result.success = true;
                result.rate = rate;
                result.lanes = lanes;
                return result;
            }
            // Fail or CR lost: fall back to the next lower rate.
            LinkRate lower;
            if (!next_lower_rate(rate, lower)) break;
            rate = lower;
        }
        if (lanes == 1) break;
    }

    aux_write(dpcd::TRAINING_PATTERN_SET, {0x00});
    result.fail_reason = "no (rate, lane count) configuration trained";
    return result;
}

// ---------------------------------------------------------------- phases

SourceTrainer::Phase SourceTrainer::clock_recovery(uint8_t lanes) {
    for (auto& d : drive_) d = cfg_.bug_start_with_high_drive ? LaneDrive{2, 2} : LaneDrive{0, 0};

    // Pattern + all lane drive settings in one burst write (0x102..0x106).
    std::vector<uint8_t> burst = {static_cast<uint8_t>(static_cast<uint8_t>(TrainingPattern::TPS1) |
                                                       SCRAMBLING_DISABLE)};
    for (int l = 0; l < lanes; ++l) burst.push_back(encode_lane_set(drive_[l]));
    if (!aux_write(dpcd::TRAINING_PATTERN_SET, burst)) return Phase::AuxError;

    uint8_t last_swing = drive_[0].swing;
    int same_swing_tries = 0;

    for (int loop = 0; loop < kMaxCrLoops; ++loop) {
        wait_read_interval(cr_read_interval_us());
        std::vector<uint8_t> status;
        if (!read_status(status)) return Phase::AuxError;

        bool all_cr = true;
        for (int l = 0; l < lanes; ++l) all_cr &= lane_has(status, l, LANE_CR_DONE);
        if (all_cr) return Phase::Ok;

        // Give up if every lane is at max swing, or we keep repeating the same swing.
        bool all_max = true;
        for (int l = 0; l < lanes; ++l) all_max &= (drive_[l].swing == MAX_LEVEL);
        if (all_max) return Phase::Fail;

        apply_adjust(status, lanes);
        if (drive_[0].swing == last_swing) {
            if (++same_swing_tries >= kMaxSameSwingTries) return Phase::Fail;
        } else {
            same_swing_tries = 0;
            last_swing = drive_[0].swing;
        }
        if (!write_drive(lanes)) return Phase::AuxError;
    }
    return Phase::Fail;
}

SourceTrainer::Phase SourceTrainer::channel_equalization(uint8_t lanes, LinkRate rate) {
    TrainingPattern tp = TrainingPattern::TPS2;
    uint8_t pattern_byte;
    if (rate == LinkRate::HBR3 && sink_tps4_ && cfg_.tps4_supported) {
        tp = TrainingPattern::TPS4;
        pattern_byte = static_cast<uint8_t>(tp);  // TPS4 is sent scrambled
    } else {
        if (sink_tps3_) tp = TrainingPattern::TPS3;
        pattern_byte = static_cast<uint8_t>(static_cast<uint8_t>(tp) | SCRAMBLING_DISABLE);
    }

    std::vector<uint8_t> burst = {pattern_byte};
    for (int l = 0; l < lanes; ++l) burst.push_back(encode_lane_set(drive_[l]));
    if (!aux_write(dpcd::TRAINING_PATTERN_SET, burst)) return Phase::AuxError;

    for (int loop = 0; loop < kMaxEqLoops; ++loop) {
        wait_read_interval(eq_read_interval_us(aux_rd_interval_));
        std::vector<uint8_t> status;
        if (!read_status(status)) return Phase::AuxError;

        bool all_cr = true, all_eq = true;
        for (int l = 0; l < lanes; ++l) {
            all_cr &= lane_has(status, l, LANE_CR_DONE);
            all_eq &= lane_has(status, l, LANE_CHANNEL_EQ_DONE) && lane_has(status, l, LANE_SYMBOL_LOCKED);
        }
        if (!all_cr) return Phase::CrLost;
        const bool aligned = status[2] & INTERLANE_ALIGN_DONE;
        if (all_eq && aligned) return Phase::Ok;

        apply_adjust(status, lanes);
        if (!write_drive(lanes)) return Phase::AuxError;
    }
    return Phase::Fail;
}

// ---------------------------------------------------------------- helpers

bool SourceTrainer::read_status(std::vector<uint8_t>& status) {
    return aux_read(dpcd::LANE0_1_STATUS, dpcd::STATUS_BLOCK_SIZE, status);  // 0x202..0x207
}

bool SourceTrainer::write_drive(uint8_t lanes) {
    std::vector<uint8_t> bytes;
    for (int l = 0; l < lanes; ++l) bytes.push_back(encode_lane_set(drive_[l]));
    return aux_write(dpcd::TRAINING_LANE0_SET, bytes);
}

void SourceTrainer::apply_adjust(const std::vector<uint8_t>& status, uint8_t lanes) {
    // status[4] = ADJUST_REQUEST_LANE0_1, status[5] = ADJUST_REQUEST_LANE2_3
    for (int l = 0; l < lanes; ++l) {
        const uint8_t reg = status[4 + l / 2];
        drive_[l] = decode_adjust(reg, l);
    }
}

void SourceTrainer::wait_read_interval(uint32_t us) {
    if (!cfg_.bug_skip_read_interval) aux_.wait_us(us);
}

bool SourceTrainer::lane_has(const std::vector<uint8_t>& status, int lane, uint8_t bit) {
    const uint8_t reg = status[lane / 2];  // status[0] = lanes 0/1, status[1] = lanes 2/3
    const uint8_t nibble = (lane % 2) ? (reg >> 4) : (reg & 0x0F);
    return nibble & bit;
}

}  // namespace dp
