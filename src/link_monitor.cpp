#include "link_monitor.h"

#include <algorithm>
#include <cstdio>

namespace dp {

// ============================================================ Coverage

void Coverage::define(const std::string& cp, const std::vector<std::string>& bins) {
    if (!points_.count(cp)) order_.push_back(cp);
    for (const auto& b : bins) points_[cp][b];  // creates bin with 0 hits
}

void Coverage::sample(const std::string& cp, const std::string& bin) {
    auto it = points_.find(cp);
    if (it == points_.end()) return;
    auto b = it->second.find(bin);
    if (b != it->second.end()) ++b->second;
}

void Coverage::merge(const Coverage& other) {
    for (const auto& cp : other.order_) {
        if (!points_.count(cp)) order_.push_back(cp);
        for (const auto& [bin, hits] : other.points_.at(cp)) points_[cp][bin] += hits;
    }
}

double Coverage::percent() const {
    int total = 0, hit = 0;
    for (const auto& [cp, bins] : points_)
        for (const auto& [bin, hits] : bins) {
            ++total;
            if (hits > 0) ++hit;
        }
    return total ? 100.0 * hit / total : 0.0;
}

void Coverage::report() const {
    std::printf("\nFunctional coverage\n");
    for (const auto& cp : order_) {
        const auto& bins = points_.at(cp);
        int hit = 0;
        for (const auto& [bin, hits] : bins) hit += hits > 0;
        std::printf("  %-22s %3d/%-3zu bins hit\n", cp.c_str(), hit, bins.size());
        for (const auto& [bin, hits] : bins)
            std::printf("      %-14s %s %d\n", bin.c_str(), hits ? "hit " : "MISS", hits);
    }
    std::printf("  Overall: %.1f%%\n", percent());
}

void define_link_training_coverage(Coverage& cov) {
    std::vector<std::string> configs;
    for (const char* r : {"RBR", "HBR", "HBR2", "HBR3"})
        for (const char* l : {"x1", "x2", "x4"}) configs.push_back(std::string(r) + "_" + l);
    cov.define("final_rate_x_lanes", configs);
    cov.define("cr_iterations", {"1", "2-4", "5+"});
    cov.define("eq_iterations", {"1", "2+"});
    cov.define("eq_pattern", {"TPS2", "TPS3", "TPS4"});
    cov.define("fallback", {"rate", "lane_count"});
    cov.define("cr_lost_during_eq", {"hit"});
    cov.define("aux_defer_streak", {"1", "2-3", "4+"});
    cov.define("aux_timeout", {"hit"});
}

// ============================================================ LinkMonitor

void LinkMonitor::flag(uint64_t t, const std::string& rule, const std::string& msg) {
    violations_.push_back({t, rule, msg});
}

bool LinkMonitor::lane_has(const std::vector<uint8_t>& s, int lane, uint8_t bit) const {
    const uint8_t reg = s[lane / 2];
    const uint8_t nibble = (lane % 2) ? (reg >> 4) : (reg & 0x0F);
    return nibble & bit;
}

uint32_t LinkMonitor::required_interval_us() const {
    return phase_ == Phase::ClockRecovery ? cr_read_interval_us() : eq_read_interval_us(aux_rd_interval_);
}

void LinkMonitor::on_transaction(uint64_t start_us, uint64_t end_us,
                                 const AuxRequest& req, const AuxResponse& rsp) {
    // [AUX-1] Payload length must be 1..16 bytes.
    const size_t n = (req.cmd == AuxCmd::NativeRead) ? req.len : req.data.size();
    if (n == 0 || n > AUX_MAX_BYTES)
        flag(start_us, "AUX-1", "request length " + std::to_string(n) + " outside 1..16");

    // Reply bookkeeping + coverage.
    if (rsp.reply == AuxReply::Defer) {
        ++defer_streak_;
        return;
    }
    if (defer_streak_ > 0) {
        cov_.sample("aux_defer_streak", defer_streak_ == 1 ? "1" : defer_streak_ <= 3 ? "2-3" : "4+");
        defer_streak_ = 0;
    }
    if (rsp.reply == AuxReply::Timeout) {
        cov_.sample("aux_timeout", "hit");
        return;
    }
    if (rsp.reply != AuxReply::Ack) return;

    if (req.cmd == AuxCmd::NativeRead) {
        // Capability read: learn what the sink supports.
        if (req.addr == dpcd::DPCD_REV && rsp.data.size() >= dpcd::CAPS_SIZE) {
            caps_known_ = true;
            sink_max_rate_ = rsp.data[dpcd::MAX_LINK_RATE];
            sink_max_lanes_ = rsp.data[dpcd::MAX_LANE_COUNT] & 0x1F;
            sink_tps4_ = rsp.data[dpcd::MAX_DOWNSPREAD] & 0x80;
            aux_rd_interval_ = rsp.data[dpcd::TRAINING_AUX_RD_INTERVAL];
        }
        if (req.addr == dpcd::LANE0_1_STATUS && rsp.data.size() >= 3) observe_status_read(start_us, rsp.data);
        return;
    }

    for (size_t i = 0; i < req.data.size(); ++i)
        observe_write(start_us, end_us, req.addr + static_cast<uint32_t>(i), req.data[i]);
}

void LinkMonitor::observe_write(uint64_t start_us, uint64_t end_us, uint32_t addr, uint8_t v) {
    if (addr == dpcd::LINK_BW_SET) {
        // [LT-2] Rate must be a legal code and not above the sink's maximum.
        if (!is_valid_rate_code(v))
            flag(start_us, "LT-2", "LINK_BW_SET has illegal code " + hex_bytes(&v, 1));
        else if (caps_known_ && v > sink_max_rate_)
            flag(start_us, "LT-2", "LINK_BW_SET above sink MAX_LINK_RATE");
        if (first_rate_code_ == 0) first_rate_code_ = v;
        else if (v < rate_code_) cov_.sample("fallback", "rate");
        rate_code_ = v;
        return;
    }
    if (addr == dpcd::LANE_COUNT_SET) {
        const uint8_t lanes = v & 0x1F;
        // [LT-3] Lane count must be 1, 2 or 4 and within the sink's maximum.
        if (lanes != 1 && lanes != 2 && lanes != 4)
            flag(start_us, "LT-3", "LANE_COUNT_SET = " + std::to_string(lanes) + " (must be 1, 2 or 4)");
        else if (caps_known_ && lanes > sink_max_lanes_)
            flag(start_us, "LT-3", "LANE_COUNT_SET above sink MAX_LANE_COUNT");
        if (first_lanes_ == 0) first_lanes_ = lanes;
        else if (lanes < lanes_) cov_.sample("fallback", "lane_count");
        lanes_ = lanes;
        return;
    }
    if (addr == dpcd::TRAINING_PATTERN_SET) {
        const auto tp = static_cast<TrainingPattern>(v & 0x0F);
        last_training_write_end_us_ = end_us;
        if (tp == TrainingPattern::TPS1) {
            phase_ = Phase::ClockRecovery;
            cr_complete_ = false;
            eq_complete_ = false;
            cr_reads_ = 0;
        } else if (tp == TrainingPattern::None) {
            if (phase_ == Phase::ChannelEq && eq_complete_) {
                trained_ = true;
                std::string bin = std::string(to_string(static_cast<LinkRate>(rate_code_))) + "_x" +
                                  std::to_string(lanes_);
                cov_.sample("final_rate_x_lanes", bin);
            }
            phase_ = Phase::Idle;
        } else {
            // [LT-4] Equalization patterns only after clock recovery completed.
            if (!cr_complete_)
                flag(start_us, "LT-4", std::string(to_string(tp)) + " sent before CR_DONE on all lanes");
            // [LT-5] DP 1.4: HBR3 must equalize with TPS4 when the sink supports it.
            if (rate_code_ == static_cast<uint8_t>(LinkRate::HBR3) && sink_tps4_ && tp != TrainingPattern::TPS4)
                flag(start_us, "LT-5", std::string("HBR3 equalization used ") + to_string(tp) + " instead of TPS4");
            phase_ = Phase::ChannelEq;
            eq_reads_ = 0;
            cov_.sample("eq_pattern", to_string(tp));
        }
        pattern_ = tp;
        return;
    }
    if (addr >= dpcd::TRAINING_LANE0_SET && addr <= dpcd::TRAINING_LANE0_SET + 3) {
        last_training_write_end_us_ = end_us;
        const LaneDrive d = decode_lane_set(v);
        // [LT-1] Voltage swing level + pre-emphasis level must not exceed 3.
        if (d.swing + d.preemph > MAX_LEVEL) {
            char msg[96];
            std::snprintf(msg, sizeof(msg), "lane %u drive VS%u + PE%u exceeds level 3",
                          static_cast<unsigned>(addr - dpcd::TRAINING_LANE0_SET), d.swing, d.preemph);
            flag(start_us, "LT-1", msg);
        }
    }
}

void LinkMonitor::observe_status_read(uint64_t start_us, const std::vector<uint8_t>& s) {
    if (phase_ == Phase::Idle) return;

    // [LT-6] Source must wait the training read interval before polling status.
    const uint64_t gap = start_us - last_training_write_end_us_;
    if (gap < required_interval_us()) {
        flag(start_us, "LT-6", "status read " + std::to_string(gap) + " us after training write (need " +
                                   std::to_string(required_interval_us()) + " us)");
    }

    bool all_cr = lanes_ > 0, all_eq = lanes_ > 0;
    for (int l = 0; l < lanes_; ++l) {
        all_cr &= lane_has(s, l, LANE_CR_DONE);
        all_eq &= lane_has(s, l, LANE_CHANNEL_EQ_DONE) && lane_has(s, l, LANE_SYMBOL_LOCKED);
    }

    if (phase_ == Phase::ClockRecovery) {
        ++cr_reads_;
        if (all_cr && !cr_complete_) {
            cr_complete_ = true;
            cov_.sample("cr_iterations", cr_reads_ == 1 ? "1" : cr_reads_ <= 4 ? "2-4" : "5+");
        }
    } else if (phase_ == Phase::ChannelEq) {
        ++eq_reads_;
        if (!all_cr) cov_.sample("cr_lost_during_eq", "hit");
        if (all_eq && (s[2] & INTERLANE_ALIGN_DONE) && !eq_complete_) {
            eq_complete_ = true;
            cov_.sample("eq_iterations", eq_reads_ == 1 ? "1" : "2+");
        }
    }
}

void LinkMonitor::final_check(uint64_t now_us) {
    // [LT-7] Training must not be left running at end of test.
    if (pattern_ != TrainingPattern::None)
        flag(now_us, "LT-7", "TRAINING_PATTERN_SET left at " + std::string(to_string(pattern_)));
}

// ============================================================ TransactionLogger

namespace {
std::string decode(uint32_t addr, const std::vector<uint8_t>& d) {
    if (d.empty()) return "";
    char buf[128];
    switch (addr) {
        case dpcd::DPCD_REV:
            if (d.size() < 4) break;
            std::snprintf(buf, sizeof(buf), "caps: rev %X.%X, max %s, %u lanes, TPS4 %s", d[0] >> 4, d[0] & 0xF,
                          is_valid_rate_code(d[1]) ? to_string(static_cast<LinkRate>(d[1])) : "?",
                          d[2] & 0x1F, (d[3] & 0x80) ? "yes" : "no");
            return buf;
        case dpcd::LINK_BW_SET:
            std::snprintf(buf, sizeof(buf), "set %s%s",
                          is_valid_rate_code(d[0]) ? to_string(static_cast<LinkRate>(d[0])) : "?",
                          d.size() > 1 ? (" x" + std::to_string(d[1] & 0x1F)).c_str() : "");
            return buf;
        case dpcd::TRAINING_PATTERN_SET: {
            std::string s = std::string(to_string(static_cast<TrainingPattern>(d[0] & 0xF))) +
                            ((d[0] & SCRAMBLING_DISABLE) ? " (scrambling off)" : "");
            for (size_t i = 1; i < d.size(); ++i) {
                LaneDrive ld = decode_lane_set(d[i]);
                std::snprintf(buf, sizeof(buf), " L%zu:VS%u/PE%u", i - 1, ld.swing, ld.preemph);
                s += buf;
            }
            return s;
        }
        case dpcd::TRAINING_LANE0_SET: {
            std::string s = "drive";
            for (size_t i = 0; i < d.size(); ++i) {
                LaneDrive ld = decode_lane_set(d[i]);
                std::snprintf(buf, sizeof(buf), " L%zu:VS%u/PE%u", i, ld.swing, ld.preemph);
                s += buf;
            }
            return s;
        }
        case dpcd::LANE0_1_STATUS: {
            if (d.size() < 6) break;
            std::string s = "status";
            for (int l = 0; l < 4; ++l) {
                const uint8_t nib = (l % 2) ? (d[l / 2] >> 4) : (d[l / 2] & 0xF);
                std::snprintf(buf, sizeof(buf), " L%d:%s%s", l, (nib & LANE_CR_DONE) ? "CR" : "--",
                              (nib & LANE_CHANNEL_EQ_DONE) ? "+EQ" : "");
                s += buf;
            }
            s += (d[2] & INTERLANE_ALIGN_DONE) ? " ALIGNED" : "";
            s += " | adjust req";
            for (int l = 0; l < 4; ++l) {
                const LaneDrive a = decode_adjust(d[4 + l / 2], l);
                std::snprintf(buf, sizeof(buf), " %u/%u", a.swing, a.preemph);
                s += buf;
            }
            return s;
        }
        default: break;
    }
    return "";
}
}  // namespace

void TransactionLogger::on_transaction(uint64_t start_us, uint64_t /*end_us*/,
                                       const AuxRequest& req, const AuxResponse& rsp) {
    const bool rd = req.cmd == AuxCmd::NativeRead;
    const std::vector<uint8_t>& payload = rd ? rsp.data : req.data;
    std::printf("  [%7llu us] AUX %s 0x%05X len=%-2u -> %-7s %s\n",
                static_cast<unsigned long long>(start_us), rd ? "RD" : "WR", req.addr,
                static_cast<unsigned>(rd ? req.len : req.data.size()), to_string(rsp.reply),
                rsp.reply == AuxReply::Ack ? decode(req.addr, payload).c_str() : "");
}

}  // namespace dp
