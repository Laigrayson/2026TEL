#include <Arduino.h>
#include <Servo.h>
#include <cstring>

// DBR4 CRSF TX is connected in parallel to GP1 on this third Pico.
static constexpr uint8_t CRSF_TX_UNUSED_PIN = 0;
static constexpr uint8_t CRSF_RX_PIN = 1;
static constexpr uint32_t CRSF_BAUD = 420000;

// Hobbywing Skywalker 40A V2 throttle signal (white wire).
static constexpr uint8_t ESC_SIGNAL_PIN = 7;
static constexpr int ESC_STOP_US = 1100;
static constexpr int ESC_FULL_US = 1940;
static constexpr uint32_t RC_TIMEOUT_MS = 200;

static Servo esc;
static uint8_t frame[64];
static size_t frameUsed = 0;
static int16_t channels[16];
static uint32_t lastRcMs = 0;
static uint32_t lastByteMs = 0;
static uint32_t lastLogMs = 0;
static bool haveRc = false;
static bool sawCh5Low = false;
static bool sawThrottleLow = false;
static bool enabled = false;
static int throttleUs = ESC_STOP_US;

static uint8_t crc8(const uint8_t* data, size_t count) {
    uint8_t crc = 0;
    while (count--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = static_cast<uint8_t>((crc << 1) ^ ((crc & 0x80) ? 0xD5 : 0));
    }
    return crc;
}

static int16_t channelPercent(uint16_t raw) {
    const int value = constrain((static_cast<int>(raw) - 992) * 100 / 820, -100, 100);
    return static_cast<int16_t>(abs(value) <= 3 ? 0 : value);
}

static void stopEsc() {
    enabled = false;
    throttleUs = ESC_STOP_US;
    esc.writeMicroseconds(ESC_STOP_US);
}

static void updateOutput() {
    const int ch5 = channels[4];
    const int ch10 = channels[9];

    // Global safety: CH5 must first be low after boot/link loss.
    if (ch5 < 0) {
        sawCh5Low = true;
        stopEsc();
    }

    // Throttle must also be brought fully low before the ESC can arm.
    if (ch10 <= -80) sawThrottleLow = true;

    if (!sawCh5Low || !sawThrottleLow || ch5 <= 50) {
        stopEsc();
        return;
    }

    enabled = true;
    const int bounded = constrain(ch10, -100, 100);
    throttleUs = map(bounded, -100, 100, ESC_STOP_US, ESC_FULL_US);
    esc.writeMicroseconds(throttleUs);
}

static void decodeChannels(const uint8_t* payload) {
    for (uint8_t ch = 0; ch < 16; ++ch) {
        uint16_t raw = 0;
        for (uint8_t bit = 0; bit < 11; ++bit) {
            const unsigned pos = ch * 11U + bit;
            raw |= static_cast<uint16_t>((payload[pos / 8] >> (pos % 8)) & 1U) << bit;
        }
        channels[ch] = channelPercent(raw);
    }
    lastRcMs = millis();
    haveRc = true;
    updateOutput();
}

static void receiveByte(uint8_t value) {
    const uint32_t now = millis();
    if (now - lastByteMs > 10) frameUsed = 0;
    lastByteMs = now;
    frame[frameUsed++] = value;

    while (frameUsed >= 2) {
        const size_t total = static_cast<size_t>(frame[1]) + 2;
        if (frame[0] != 0xC8 || frame[1] < 2 || frame[1] > 62) {
            memmove(frame, frame + 1, --frameUsed);
            continue;
        }
        if (frameUsed < total) break;
        if (crc8(frame + 2, total - 3) == frame[total - 1]) {
            if (frame[2] == 0x16 && frame[1] == 24) decodeChannels(frame + 3);
            frameUsed -= total;
            memmove(frame, frame + total, frameUsed);
        } else {
            memmove(frame, frame + 1, --frameUsed);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Attach and command minimum throttle before listening to the receiver.
    esc.attach(ESC_SIGNAL_PIN, 1000, 2000);
    stopEsc();

    Serial1.setTX(CRSF_TX_UNUSED_PIN);
    Serial1.setRX(CRSF_RX_PIN);
    Serial1.setFIFOSize(256);
    Serial1.begin(CRSF_BAUD);

    Serial.println("brushless_ch10: DBR4 CRSF GP1 -> ESC signal GP7");
    Serial.println("Safety: CH5 low first, CH10 fully low first, 200ms failsafe.");
}

void loop() {
    while (Serial1.available()) receiveByte(static_cast<uint8_t>(Serial1.read()));

    const uint32_t now = millis();
    if (haveRc && now - lastRcMs > RC_TIMEOUT_MS) {
        stopEsc();
        haveRc = false;
        sawCh5Low = false;
        sawThrottleLow = false;
        frameUsed = 0;
        while (Serial1.available()) Serial1.read();
    }

    if (now - lastLogMs >= 500) {
        lastLogMs = now;
        Serial.printf("DBR4=%s enabled=%d CH5=%d CH10=%d throttle=%dus\n",
                      haveRc ? "OK" : "LOST", enabled,
                      channels[4], channels[9], throttleUs);
    }
}

