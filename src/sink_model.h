// Behavioral model of a DP sink (receiver) for 8b/10b link training.
//
// The physical channel is abstracted: for each lane at each link rate, the sink
// needs a minimum voltage swing to achieve clock recovery and a minimum
// pre-emphasis to achieve equalization. If swing + pre-emphasis would exceed 3,
// that lane can never train at that rate and the source must fall back.
#pragma once

#include <array>
#include <cstdint>

#include "aux_channel.h"
#include "dpcd.h"

namespace dp {

struct LaneChannel {
    uint8_t cr_swing = 0;    // swing needed for CR_DONE
    uint8_t eq_preemph = 0;  // pre-emphasis needed for CHANNEL_EQ_DONE
};

struct SinkConfig {
    uint8_t dpcd_rev = 0x14;              // DPCD 1.4
    LinkRate max_rate = LinkRate::HBR3;
    uint8_t max_lanes = 4;
    bool tps3_supported = true;
    bool tps4_supported = true;
    uint8_t aux_rd_interval = 0;          // 0 -> 400 us EQ interval

    // channel[rate_index][lane]
    std::array<std::array<LaneChannel, 4>, 4> channel{};

    // ---- error injection ----
    int defer_next = 0;                   // reply DEFER to the next N requests
    int timeout_next = 0;                 // stay silent for the next N requests
    bool drop_cr_once_during_eq = false;  // clear CR_DONE on lane 0 once during EQ
    int cr_hold_updates = 0;              // withhold CR_DONE for N status updates (slow lock)
};

class SinkModel : public AuxResponder {
public:
    explicit SinkModel(const SinkConfig& cfg);
    AuxResponse handle(const AuxRequest& req, uint64_t now_us) override;

    uint8_t peek(uint32_t addr) const { return dpcd_[addr]; }

private:
    void update_training_status();
    LinkRate current_rate() const;
    uint8_t active_lanes() const;

    SinkConfig cfg_;
    std::array<uint8_t, dpcd::SPACE_SIZE> dpcd_{};
    int defers_left_ = 0;
    int timeouts_left_ = 0;
    bool cr_drop_pending_ = false;
    int cr_hold_left_ = 0;
};

}  // namespace dp
