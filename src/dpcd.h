// DPCD register map subset and bit-field helpers used by 8b/10b link training.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace dp {

namespace dpcd {
// Receiver capability
constexpr uint32_t DPCD_REV                 = 0x00000;
constexpr uint32_t MAX_LINK_RATE            = 0x00001;
constexpr uint32_t MAX_LANE_COUNT           = 0x00002;  // [4:0] lanes, [6] TPS3, [7] enhanced framing
constexpr uint32_t MAX_DOWNSPREAD           = 0x00003;  // [7] TPS4 supported
constexpr uint32_t TRAINING_AUX_RD_INTERVAL = 0x0000E;  // [6:0] EQ read interval code
constexpr uint32_t CAPS_SIZE                = 0x10;     // 0x00000..0x0000F

// Link configuration
constexpr uint32_t LINK_BW_SET              = 0x00100;
constexpr uint32_t LANE_COUNT_SET           = 0x00101;
constexpr uint32_t TRAINING_PATTERN_SET     = 0x00102;  // [3:0] pattern, [5] scrambling disable
constexpr uint32_t TRAINING_LANE0_SET       = 0x00103;  // 0x00103..0x00106, one byte per lane

// Link / sink status
constexpr uint32_t LANE0_1_STATUS           = 0x00202;  // lane0 = [3:0], lane1 = [7:4]
constexpr uint32_t LANE2_3_STATUS           = 0x00203;
constexpr uint32_t LANE_ALIGN_STATUS        = 0x00204;  // [0] INTERLANE_ALIGN_DONE
constexpr uint32_t SINK_STATUS              = 0x00205;
constexpr uint32_t ADJUST_REQUEST_LANE0_1   = 0x00206;  // lane0 VS[1:0] PE[3:2], lane1 VS[5:4] PE[7:6]
constexpr uint32_t ADJUST_REQUEST_LANE2_3   = 0x00207;
constexpr uint32_t STATUS_BLOCK_SIZE        = 6;        // 0x00202..0x00207

constexpr uint32_t SPACE_SIZE               = 0x00800;  // modeled address range
}  // namespace dpcd

// Per-lane status nibble bits (in LANEx_y_STATUS)
constexpr uint8_t LANE_CR_DONE         = 0x1;
constexpr uint8_t LANE_CHANNEL_EQ_DONE = 0x2;
constexpr uint8_t LANE_SYMBOL_LOCKED   = 0x4;

constexpr uint8_t SCRAMBLING_DISABLE   = 0x20;
constexpr uint8_t INTERLANE_ALIGN_DONE = 0x01;

constexpr uint8_t MAX_LEVEL = 3;  // swing and pre-emphasis levels run 0..3

// 8b/10b link rate codes: value = rate / 0.27 Gbps
enum class LinkRate : uint8_t { RBR = 0x06, HBR = 0x0A, HBR2 = 0x14, HBR3 = 0x1E };

enum class TrainingPattern : uint8_t { None = 0, TPS1 = 1, TPS2 = 2, TPS3 = 3, TPS4 = 7 };

struct LaneDrive {
    uint8_t swing = 0;    // voltage swing level 0..3
    uint8_t preemph = 0;  // pre-emphasis level 0..3
};

const char* to_string(LinkRate r);
const char* to_string(TrainingPattern p);
double gbps(LinkRate r);
bool is_valid_rate_code(uint8_t code);
int rate_index(LinkRate r);              // RBR=0 .. HBR3=3
bool next_lower_rate(LinkRate r, LinkRate& out);

// TRAINING_LANEx_SET: VS[1:0], MAX_SWING_REACHED[2], PE[4:3], MAX_PRE_EMPHASIS_REACHED[5]
uint8_t encode_lane_set(LaneDrive d);
LaneDrive decode_lane_set(uint8_t v);

// ADJUST_REQUEST_LANEx_y: two lanes per byte
uint8_t encode_adjust_pair(LaneDrive even_lane, LaneDrive odd_lane);
LaneDrive decode_adjust(uint8_t reg, int lane);  // lane parity selects the half

// Read interval (microseconds) the source must wait before reading training status.
uint32_t cr_read_interval_us();
uint32_t eq_read_interval_us(uint8_t aux_rd_interval_reg);

std::string hex_bytes(const uint8_t* data, size_t n);

}  // namespace dp
