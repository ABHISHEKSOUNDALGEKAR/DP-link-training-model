// DP source (transmitter) link-training state machine for 8b/10b rates.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "aux_channel.h"
#include "dpcd.h"

namespace dp {

struct SourceConfig {
    LinkRate max_rate = LinkRate::HBR3;
    uint8_t max_lanes = 4;
    bool tps4_supported = true;
    int aux_max_retries = 7;          // retries after DEFER or timeout

    // ---- deliberate bugs, used to prove the checker catches them ----
    bool bug_start_with_high_drive = false;  // starts CR at swing 2 + pre-emphasis 2 (illegal)
    bool bug_skip_read_interval = false;     // polls status without waiting
};

struct TrainResult {
    bool success = false;
    LinkRate rate = LinkRate::RBR;
    uint8_t lanes = 0;
    int attempts = 0;          // (rate, lane count) configurations tried
    std::string fail_reason;
};

class SourceTrainer {
public:
    SourceTrainer(AuxChannel& aux, const SourceConfig& cfg) : aux_(aux), cfg_(cfg) {}
    TrainResult train();

private:
    enum class Phase { Ok, Fail, CrLost, AuxError };

    bool aux_read(uint32_t addr, uint8_t len, std::vector<uint8_t>& out);
    bool aux_write(uint32_t addr, const std::vector<uint8_t>& data);

    Phase clock_recovery(uint8_t lanes);
    Phase channel_equalization(uint8_t lanes, LinkRate rate);
    bool read_status(std::vector<uint8_t>& status);
    bool write_drive(uint8_t lanes);
    void apply_adjust(const std::vector<uint8_t>& status, uint8_t lanes);
    void wait_read_interval(uint32_t us);

    static bool lane_has(const std::vector<uint8_t>& status, int lane, uint8_t bit);

    AuxChannel& aux_;
    SourceConfig cfg_;
    LaneDrive drive_[4];
    bool sink_tps3_ = false;
    bool sink_tps4_ = false;
    uint8_t aux_rd_interval_ = 0;
    std::string last_error_;
};

}  // namespace dp
