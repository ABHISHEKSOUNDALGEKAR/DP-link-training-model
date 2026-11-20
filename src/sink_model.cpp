#include "sink_model.h"

#include <algorithm>

namespace dp {

SinkModel::SinkModel(const SinkConfig& cfg)
    : cfg_(cfg),
      defers_left_(cfg.defer_next),
      timeouts_left_(cfg.timeout_next),
      cr_drop_pending_(cfg.drop_cr_once_during_eq),
      cr_hold_left_(cfg.cr_hold_updates) {
    dpcd_[dpcd::DPCD_REV] = cfg.dpcd_rev;
    dpcd_[dpcd::MAX_LINK_RATE] = static_cast<uint8_t>(cfg.max_rate);
    dpcd_[dpcd::MAX_LANE_COUNT] =
        static_cast<uint8_t>((cfg.max_lanes & 0x1F) | (cfg.tps3_supported ? 0x40 : 0) | 0x80);
    dpcd_[dpcd::MAX_DOWNSPREAD] = cfg.tps4_supported ? 0x80 : 0x00;
    dpcd_[dpcd::TRAINING_AUX_RD_INTERVAL] = cfg.aux_rd_interval;
}

AuxResponse SinkModel::handle(const AuxRequest& req, uint64_t /*now_us*/) {
    AuxResponse rsp;

    if (timeouts_left_ > 0) {   // injected: no reply at all
        --timeouts_left_;
        rsp.reply = AuxReply::Timeout;
        return rsp;
    }
    if (defers_left_ > 0) {     // injected: "busy, try again"
        --defers_left_;
        rsp.reply = AuxReply::Defer;
        return rsp;
    }

    const size_t n = (req.cmd == AuxCmd::NativeRead) ? req.len : req.data.size();
    if (n == 0 || n > AUX_MAX_BYTES || req.addr + n > dpcd::SPACE_SIZE) {
        rsp.reply = AuxReply::Nack;
        return rsp;
    }

    if (req.cmd == AuxCmd::NativeRead) {
        rsp.data.assign(dpcd_.begin() + req.addr, dpcd_.begin() + req.addr + n);
    } else {
        // A burst write updates consecutive registers; react once after all bytes land.
        bool training_touched = false;
        for (size_t i = 0; i < n; ++i) {
            const uint32_t a = req.addr + static_cast<uint32_t>(i);
            dpcd_[a] = req.data[i];
            if (a >= dpcd::LINK_BW_SET && a <= dpcd::TRAINING_LANE0_SET + 3) training_touched = true;
        }
        if (training_touched) update_training_status();
    }
    rsp.reply = AuxReply::Ack;
    return rsp;
}

LinkRate SinkModel::current_rate() const {
    const uint8_t code = dpcd_[dpcd::LINK_BW_SET];
    return is_valid_rate_code(code) ? static_cast<LinkRate>(code) : LinkRate::RBR;
}

uint8_t SinkModel::active_lanes() const { return dpcd_[dpcd::LANE_COUNT_SET] & 0x1F; }

// Recompute lane status and adjust requests from the current pattern and drive settings.
void SinkModel::update_training_status() {
    const auto pattern = static_cast<TrainingPattern>(dpcd_[dpcd::TRAINING_PATTERN_SET] & 0x0F);
    const LinkRate rate = current_rate();
    const int ri = rate_index(rate);
    const uint8_t lanes = std::min<uint8_t>(active_lanes(), 4);

    uint8_t status[4] = {0, 0, 0, 0};
    LaneDrive adjust[4];
    bool all_eq = lanes > 0;

    // DP 1.4: HBR3 requires TPS4 for equalization when the sink supports it.
    const bool eq_pattern_ok = (rate != LinkRate::HBR3 || !cfg_.tps4_supported ||
                                pattern == TrainingPattern::TPS4);

    for (int lane = 0; lane < lanes; ++lane) {
        const LaneChannel need = cfg_.channel[ri][lane];
        const LaneDrive cur = decode_lane_set(dpcd_[dpcd::TRAINING_LANE0_SET + lane]);

        const bool in_eq = pattern != TrainingPattern::TPS1 && pattern != TrainingPattern::None;

        // Adjust requests: during CR, step the swing up one level at a time; during EQ,
        // ask for the pre-emphasis the channel needs. Never request swing + PE > 3.
        adjust[lane].swing = (cur.swing < need.cr_swing) ? static_cast<uint8_t>(cur.swing + 1) : cur.swing;
        const uint8_t want_pe = in_eq ? need.eq_preemph : cur.preemph;
        adjust[lane].preemph =
            std::min<uint8_t>(want_pe, static_cast<uint8_t>(MAX_LEVEL - adjust[lane].swing));

        if (pattern == TrainingPattern::None) continue;

        bool cr = cur.swing >= need.cr_swing;
        if (!in_eq && cr_hold_left_ > 0) {  // injected: slow CDR lock, withhold CR_DONE
            cr = false;
            if (lane == lanes - 1) --cr_hold_left_;
        }
        if (in_eq && lane == 0 && cr_drop_pending_) {  // injected: lose clock recovery once
            cr = false;
            cr_drop_pending_ = false;
        }
        if (cr) status[lane] |= LANE_CR_DONE;

        if (in_eq && cr && eq_pattern_ok && cur.preemph >= need.eq_preemph) {
            status[lane] |= LANE_CHANNEL_EQ_DONE | LANE_SYMBOL_LOCKED;
        } else {
            all_eq = false;
        }
    }

    dpcd_[dpcd::LANE0_1_STATUS] = static_cast<uint8_t>(status[0] | (status[1] << 4));
    dpcd_[dpcd::LANE2_3_STATUS] = static_cast<uint8_t>(status[2] | (status[3] << 4));
    dpcd_[dpcd::LANE_ALIGN_STATUS] = (pattern != TrainingPattern::TPS1 && all_eq) ? INTERLANE_ALIGN_DONE : 0;
    dpcd_[dpcd::ADJUST_REQUEST_LANE0_1] = encode_adjust_pair(adjust[0], adjust[1]);
    dpcd_[dpcd::ADJUST_REQUEST_LANE2_3] = encode_adjust_pair(adjust[2], adjust[3]);
}

}  // namespace dp
