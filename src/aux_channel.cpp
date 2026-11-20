#include "aux_channel.h"

namespace dp {

const char* to_string(AuxReply r) {
    switch (r) {
        case AuxReply::Ack:     return "ACK";
        case AuxReply::Nack:    return "NACK";
        case AuxReply::Defer:   return "DEFER";
        case AuxReply::Timeout: return "TIMEOUT";
    }
    return "?";
}

// Rough timing at 1 Mbps Manchester-II: ~1 us per bit, plus preamble/sync overhead.
// Request header is 4 bytes (cmd + 20-bit addr + len); reply header is 1 byte.
uint64_t AuxChannel::duration_us(const AuxRequest& req, const AuxResponse& rsp) {
    constexpr uint64_t kOverheadUs = 30;     // preamble + sync + stop, each direction
    constexpr uint64_t kTimeoutUs = 400;     // source gives up waiting for a reply
    uint64_t t = kOverheadUs + 8 * (4 + req.data.size());
    if (rsp.reply == AuxReply::Timeout) return t + kTimeoutUs;
    return t + kOverheadUs + 8 * (1 + rsp.data.size());
}

AuxResponse AuxChannel::transact(const AuxRequest& req) {
    const uint64_t start = now_us_;
    AuxResponse rsp = responder_.handle(req, start);
    now_us_ += duration_us(req, rsp);
    for (AuxObserver* obs : observers_) obs->on_transaction(start, now_us_, req, rsp);
    return rsp;
}

}  // namespace dp
