#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include "rc_control_link.h"

// Dedicated stepper GP0 TX -> chassis GP5 RX, 115200 8N1.
// D3 91 version valid distanceLo distanceHi sequence crcLo crcHi.
namespace UltrasonicLink {
constexpr uint32_t BAUD = 115200;
constexpr uint32_t TIMEOUT_MS = 350;
constexpr int STOP_CM = RcControlLink::OBSTACLE_CM;
constexpr int RELEASE_CM = STOP_CM + 1;
constexpr size_t PACKET_SIZE = 9;

inline uint16_t crc(const uint8_t* data, size_t count) {
    uint16_t value = 0xFFFF;
    while(count--) {
        value ^= static_cast<uint16_t>(*data++) << 8;
        for(uint8_t bit = 0; bit < 8; ++bit)
            value = static_cast<uint16_t>((value << 1) ^
                                        ((value & 0x8000) ? 0x1021 : 0));
    }
    return value;
}

inline void encode(uint8_t* packet, bool valid, int distanceCm, uint8_t sequence) {
    const uint16_t distance = valid ? static_cast<uint16_t>(distanceCm) : 0;
    packet[0] = 0xD3;
    packet[1] = 0x91;
    packet[2] = 1;
    packet[3] = valid ? 1 : 0;
    packet[4] = static_cast<uint8_t>(distance);
    packet[5] = static_cast<uint8_t>(distance >> 8);
    packet[6] = sequence;
    const uint16_t checksum = crc(packet + 2, 5);
    packet[7] = static_cast<uint8_t>(checksum);
    packet[8] = static_cast<uint8_t>(checksum >> 8);
}

struct Receiver {
    uint8_t window[PACKET_SIZE] = {};
    size_t used = 0;
    uint32_t lastPacketMs = 0;
    bool havePacket = false;
    bool valid = false;
    bool blocked = true;
    int distanceCm = -1;

    bool fresh(uint32_t nowMs) const {
        return havePacket && nowMs - lastPacketMs <= TIMEOUT_MS;
    }

    void expire(uint32_t nowMs) {
        if(!fresh(nowMs)) {
            valid = false;
            blocked = true;
            distanceCm = -1;
        }
    }

    bool feed(uint8_t byte, uint32_t nowMs) {
        window[used++] = byte;
        if(used < PACKET_SIZE) return false;
        const int distance = window[4] | (static_cast<uint16_t>(window[5]) << 8);
        const uint16_t received = window[7] | (static_cast<uint16_t>(window[8]) << 8);
        const bool good = window[0] == 0xD3 && window[1] == 0x91 &&
            window[2] == 1 && window[3] <= 1 &&
            (window[3] == 0 ? distance == 0 : distance >= 2 && distance <= 400) &&
            crc(window + 2, 5) == received;
        if(!good) {
            std::memmove(window, window + 1, PACKET_SIZE - 1);
            used = PACKET_SIZE - 1;
            return false;
        }
        used = 0;
        havePacket = true;
        lastPacketMs = nowMs;
        valid = window[3] != 0;
        distanceCm = valid ? distance : -1;
        if(!valid || distance <= STOP_CM) blocked = true;
        else if(distance >= RELEASE_CM) blocked = false;
        return true;
    }
};
} // namespace UltrasonicLink
