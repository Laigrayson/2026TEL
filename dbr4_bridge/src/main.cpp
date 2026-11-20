#include <Arduino.h>
#include <cstring>
#include "../../common/rc_control_link.h"

// DBR4 TX -> GP1 (UART0 RX). Inter-Pico link: GP4 TX / GP5 RX.
// GP4 TX is shared by the motor Pico and the stepper Pico (both listen on GP1).
// Link packet v3, 16 bytes. CRC covers bytes 2..13.
static uint8_t frame[64];
static size_t used = 0;
static int16_t channels[16];
static uint32_t lastRcMs = 0;
static uint32_t lastByteMs = 0;
static uint32_t lastSendMs = 0;
static uint32_t lastLogMs = 0;
static bool haveRc = false;
static bool sawDisarmed = false;
static bool enabled = false;
static uint8_t sequence = 0;

static uint8_t crc8(const uint8_t* data, size_t count) {
    uint8_t crc = 0;
    while (count--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = static_cast<uint8_t>((crc << 1) ^ ((crc & 0x80) ? 0xD5 : 0));
    }
    return crc;
}

static uint16_t crc16(const uint8_t* data, size_t count) {
    uint16_t crc = 0xFFFF;
    while (count--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = static_cast<uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}

static int16_t axis(uint16_t raw, bool useDeadzone) {
    const int value = constrain((static_cast<int>(raw) - 992) * 100 / 820, -100, 100);
    return static_cast<int16_t>(useDeadzone && abs(value) <= 10 ? 0 : value);
}

static void decodeChannels(const uint8_t* payload) {
    for (uint8_t ch = 0; ch < 16; ++ch) {
        uint16_t raw = 0;
        for (uint8_t bit = 0; bit < 11; ++bit) {
            const unsigned pos = ch * 11U + bit;
            raw |= static_cast<uint16_t>((payload[pos / 8] >> (pos % 8)) & 1U) << bit;
        }
        channels[ch] = axis(raw, ch != 9); // CH10 knob needs continuous travel.
    }
    lastRcMs = millis();
    haveRc = true;
    // CH7 low disarms. After startup or loss, cycle through low and center the sticks.
    if (channels[6] <= RcControlLink::SAFETY_STOP_MAX) {
        sawDisarmed = true;
        enabled = false;
    } else if (sawDisarmed && channels[0] == 0 && channels[1] == 0 && channels[3] == 0) {
        enabled = true;
    }
}

static void receiveByte(uint8_t value) {
    const uint32_t now = millis();
    if (now - lastByteMs > 10) used = 0;
    lastByteMs = now;
    frame[used++] = value;
    // Sliding window resynchronizes after corrupt or truncated frames.
    while (used >= 2) {
        const size_t total = static_cast<size_t>(frame[1]) + 2;
        if (frame[0] != 0xC8 || frame[1] < 2 || frame[1] > 62) {
            memmove(frame, frame + 1, --used);
            continue;
        }
        if (used < total) break;
        if (crc8(frame + 2, total - 3) == frame[total - 1]) {
            if (frame[2] == 0x16 && frame[1] == 24) decodeChannels(frame + 3);
            used -= total;
            memmove(frame, frame + total, used);
        } else {
            memmove(frame, frame + 1, --used);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("dbr4_bridge: RP2040 receiver bridge started");
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.setFIFOSize(256);
    Serial1.begin(420000);
    Serial2.setTX(4);
    Serial2.setRX(5);
    Serial2.setFIFOSize(256);
    Serial2.begin(115200);
}

void loop() {
    while (Serial1.available()) receiveByte(static_cast<uint8_t>(Serial1.read()));
    while (Serial2.available()) Serial2.read();
    const uint32_t now = millis();
    const bool connected = haveRc && now - lastRcMs <= 200;
    if (!connected) {
        enabled = false;
        sawDisarmed = false;
    }
    if (now - lastSendMs >= 20) {
        lastSendMs = now;
        uint8_t packet[RcControlLink::PACKET_SIZE] = {
            0xA5, 0x5A, RcControlLink::VERSION, sequence++, static_cast<uint8_t>(enabled)};
        packet[8] = static_cast<uint8_t>(static_cast<int8_t>(connected ? channels[9] : -100));
        packet[9] = static_cast<uint8_t>(static_cast<int8_t>(connected ? channels[6] : -100));
        // CH9 is the momentary TT-gate button.  Forward it even while CH5 is
        // disarmed; a lost DBR4 link explicitly sends the released value.
        packet[10] = static_cast<uint8_t>(static_cast<int8_t>(connected ? channels[8] : -100));
        // CH8 is the stepper button.  Forward it while the link is healthy so
        // the stepper Pico can see both +100 and -100.  A lost link sends -100.
        packet[11] = static_cast<uint8_t>(static_cast<int8_t>(connected ? channels[7] : -100));
        if (enabled) {
            packet[5] = static_cast<uint8_t>(static_cast<int8_t>(channels[0])); // CH1 right
            packet[6] = static_cast<uint8_t>(static_cast<int8_t>(channels[1])); // CH2 forward
            packet[7] = static_cast<uint8_t>(static_cast<int8_t>(channels[3])); // CH4 yaw
        }
        packet[12] = static_cast<uint8_t>(static_cast<int8_t>(connected ? channels[4] : 100));
        packet[13] = static_cast<uint8_t>(static_cast<int8_t>(connected ? channels[5] : 0));
        const uint16_t crc = crc16(packet + 2, 12);
        packet[14] = static_cast<uint8_t>(crc);
        packet[15] = static_cast<uint8_t>(crc >> 8);
        Serial2.write(packet, sizeof(packet));
    }
    if (now - lastLogMs >= 1000) {
        lastLogMs = now;
        Serial.printf("DBR4=%s enabled=%d CH1=%d CH2=%d CH4=%d CH5=%d CH6=%d CH7=%d CH8=%d CH9=%d CH10=%d\n",
                      connected ? "OK" : "LOST", enabled, channels[0], channels[1],
                      channels[3], channels[4], channels[5], channels[6], channels[7], channels[8], channels[9]);
    }
}
