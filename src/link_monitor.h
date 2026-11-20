// Passive monitor for the AUX channel. It reconstructs link-training state purely
// from observed transactions (like a VIP monitor watching the wires), then:
//   - runs protocol checks and records violations
//   - samples functional coverage
// A separate logger prints decoded transactions for debugging.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "aux_channel.h"
#include "dpcd.h"

namespace dp {

struct Violation {
    uint64_t time_us;
    std::string rule;
    std::string message;
};

// Minimal functional-coverage model: coverpoints with named bins and hit counts.
class Coverage {
public:
    void define(const std::string& coverpoint, const std::vector<std::string>& bins);
    void sample(const std::string& coverpoint, const std::string& bin);
    void merge(const Coverage& other);
    void report() const;
    double percent() const;

private:
    std::map<std::string, std::map<std::string, int>> points_;
    std::vector<std::string> order_;
};

void define_link_training_coverage(Coverage& cov);

class LinkMonitor : public AuxObserver {
public:
    explicit LinkMonitor(Coverage& cov) : cov_(cov) {}

    void on_transaction(uint64_t start_us, uint64_t end_us,
                        const AuxRequest& req, const AuxResponse& rsp) override;

    // End-of-test checks (like a UVM check phase).
    void final_check(uint64_t now_us);

    const std::vector<Violation>& violations() const { return violations_; }

private:
    enum class Phase { Idle, ClockRecovery, ChannelEq };

    void observe_write(uint64_t start_us, uint64_t end_us, uint32_t addr, uint8_t value);
    void observe_status_read(uint64_t start_us, const std::vector<uint8_t>& status);
    void flag(uint64_t t, const std::string& rule, const std::string& msg);
    uint32_t required_interval_us() const;
    bool lane_has(const std::vector<uint8_t>& s, int lane, uint8_t bit) const;

    Coverage& cov_;
    std::vector<Violation> violations_;

    // Sink capabilities learned from the capability read.
    bool caps_known_ = false;
    uint8_t sink_max_rate_ = 0;
    uint8_t sink_max_lanes_ = 0;
    bool sink_tps4_ = false;
    uint8_t aux_rd_interval_ = 0;

    // Link state reconstructed from writes.
    uint8_t rate_code_ = 0;
    uint8_t lanes_ = 0;
    uint8_t first_rate_code_ = 0;
    uint8_t first_lanes_ = 0;
    Phase phase_ = Phase::Idle;
    TrainingPattern pattern_ = TrainingPattern::None;
    bool cr_complete_ = false;
    bool eq_complete_ = false;
    int cr_reads_ = 0;
    int eq_reads_ = 0;
    uint64_t last_training_write_end_us_ = 0;
    int defer_streak_ = 0;
    bool trained_ = false;
};

// Prints each AUX transaction with a short decode of well-known registers.
class TransactionLogger : public AuxObserver {
public:
    void on_transaction(uint64_t start_us, uint64_t end_us,
                        const AuxRequest& req, const AuxResponse& rsp) override;
};

}  // namespace dp
