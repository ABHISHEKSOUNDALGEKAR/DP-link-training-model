// Transaction-level model of the DP AUX channel (native AUX only, no I2C-over-AUX).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dp {

constexpr size_t AUX_MAX_BYTES = 16;  // max payload per AUX transaction

enum class AuxCmd : uint8_t { NativeWrite = 0x8, NativeRead = 0x9 };
enum class AuxReply : uint8_t { Ack = 0x0, Nack = 0x1, Defer = 0x2, Timeout = 0xFF };

struct AuxRequest {
    AuxCmd cmd = AuxCmd::NativeRead;
    uint32_t addr = 0;            // 20-bit DPCD address
    uint8_t len = 0;              // bytes to read (reads) or data.size() (writes)
    std::vector<uint8_t> data;    // write payload
};

struct AuxResponse {
    AuxReply reply = AuxReply::Ack;
    std::vector<uint8_t> data;    // read payload
};

const char* to_string(AuxReply r);

// The device on the far side of the AUX channel (the sink, or an LTTPR).
class AuxResponder {
public:
    virtual ~AuxResponder() = default;
    virtual AuxResponse handle(const AuxRequest& req, uint64_t now_us) = 0;
};

// Anything that passively watches the AUX channel (monitor, checker, coverage, logger).
class AuxObserver {
public:
    virtual ~AuxObserver() = default;
    virtual void on_transaction(uint64_t start_us, uint64_t end_us,
                                const AuxRequest& req, const AuxResponse& rsp) = 0;
};

// Connects a requester (the source) to a responder (the sink), keeps simulated
// time, and broadcasts every completed transaction to the observers.
class AuxChannel {
public:
    explicit AuxChannel(AuxResponder& responder) : responder_(responder) {}

    void add_observer(AuxObserver* obs) { observers_.push_back(obs); }
    AuxResponse transact(const AuxRequest& req);
    void wait_us(uint64_t us) { now_us_ += us; }
    uint64_t now_us() const { return now_us_; }

private:
    static uint64_t duration_us(const AuxRequest& req, const AuxResponse& rsp);

    AuxResponder& responder_;
    std::vector<AuxObserver*> observers_;
    uint64_t now_us_ = 0;
};

}  // namespace dp
