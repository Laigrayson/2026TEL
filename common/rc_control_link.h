#pragma once

#include <stddef.h>
#include <stdint.h>

namespace RcControlLink {
// A5 5A version seq enabled x y yaw CH10 CH7 CH9 CH8 CH5 CH6 crcLo crcHi
constexpr uint8_t VERSION = 3;
constexpr size_t PACKET_SIZE = 16;
constexpr int SAFETY_STOP_MAX = -50;
constexpr int BYPASS_MIN = 50;
constexpr int OBSTACLE_CM = 173;
}
