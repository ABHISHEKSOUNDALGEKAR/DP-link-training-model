#include "dpcd.h"

#include <cstdio>

namespace dp {

const char* to_string(LinkRate r) {
    switch (r) {
        case LinkRate::RBR:  return "RBR";
        case LinkRate::HBR:  return "HBR";
        case LinkRate::HBR2: return "HBR2";
        case LinkRate::HBR3: return "HBR3";
    }
    return "?";
}

const char* to_string(TrainingPattern p) {
    switch (p) {
        case TrainingPattern::None: return "NONE";
        case TrainingPattern::TPS1: return "TPS1";
        case TrainingPattern::TPS2: return "TPS2";
        case TrainingPattern::TPS3: return "TPS3";
        case TrainingPattern::TPS4: return "TPS4";
    }
    return "?";
}

double gbps(LinkRate r) { return static_cast<uint8_t>(r) * 0.27; }

bool is_valid_rate_code(uint8_t code) {
    return code == 0x06 || code == 0x0A || code == 0x14 || code == 0x1E;
}

int rate_index(LinkRate r) {
    switch (r) {
        case LinkRate::RBR:  return 0;
        case LinkRate::HBR:  return 1;
        case LinkRate::HBR2: return 2;
        case LinkRate::HBR3: return 3;
    }
    return 0;
}

bool next_lower_rate(LinkRate r, LinkRate& out) {
    switch (r) {
        case LinkRate::HBR3: out = LinkRate::HBR2; return true;
        case LinkRate::HBR2: out = LinkRate::HBR;  return true;
        case LinkRate::HBR:  out = LinkRate::RBR;  return true;
        case LinkRate::RBR:  return false;
    }
    return false;
}

uint8_t encode_lane_set(LaneDrive d) {
    uint8_t v = static_cast<uint8_t>((d.swing & 0x3) | ((d.preemph & 0x3) << 3));
    if (d.swing == MAX_LEVEL) v |= 0x04;    // MAX_SWING_REACHED
    if (d.preemph == MAX_LEVEL) v |= 0x20;  // MAX_PRE_EMPHASIS_REACHED
    return v;
}

LaneDrive decode_lane_set(uint8_t v) {
    LaneDrive d;
    d.swing = v & 0x3;
    d.preemph = (v >> 3) & 0x3;
    return d;
}

uint8_t encode_adjust_pair(LaneDrive even_lane, LaneDrive odd_lane) {
    return static_cast<uint8_t>((even_lane.swing & 0x3) | ((even_lane.preemph & 0x3) << 2) |
                                ((odd_lane.swing & 0x3) << 4) | ((odd_lane.preemph & 0x3) << 6));
}

LaneDrive decode_adjust(uint8_t reg, int lane) {
    const int shift = (lane % 2) ? 4 : 0;
    LaneDrive d;
    d.swing = (reg >> shift) & 0x3;
    d.preemph = (reg >> (shift + 2)) & 0x3;
    return d;
}

uint32_t cr_read_interval_us() { return 100; }

uint32_t eq_read_interval_us(uint8_t aux_rd_interval_reg) {
    const uint8_t code = aux_rd_interval_reg & 0x7F;
    return code == 0 ? 400 : static_cast<uint32_t>(code) * 4000;  // 0 -> 400 us, n -> n * 4 ms
}

std::string hex_bytes(const uint8_t* data, size_t n) {
    std::string s;
    char buf[4];
    for (size_t i = 0; i < n; ++i) {
        std::snprintf(buf, sizeof(buf), "%02X", data[i]);
        if (i) s += ' ';
        s += buf;
    }
    return s;
}

}  // namespace dp
