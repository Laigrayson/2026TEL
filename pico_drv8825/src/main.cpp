// TMC2209 + 42 mm stepper (NEMA 17) in standalone STEP/DIR mode.
// Wiring: GP2->STEP, GP3->DIR, GP4->EN (active LOW), MS1=GND, MS2=GND,
// CLK=GND. TMC2209 UART is deliberately not used in this first test version.

#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include "../../common/ultrasonic_link.h"
#include "../../common/rc_control_link.h"

namespace Pin {
constexpr uint8_t STEP = 2;
constexpr uint8_t DIR = 3;
constexpr uint8_t ENABLE = 4; // TMC2209 ENN: LOW = enabled
constexpr uint8_t ULTRASONIC_TRIG = 5;
constexpr uint8_t ULTRASONIC_ECHO = 6;
// Reversed relative to the previous firmware (which used LOW for forward).
constexpr uint8_t DIR_FORWARD_LEVEL = HIGH;
}

namespace Motor {
constexpr uint16_t FULL_STEPS_PER_REV = 200;
constexpr uint8_t MICROSTEPS = 8; // MS1=GND, MS2=GND
constexpr float MAX_RPM = 60.0f;
constexpr float MAX_STEP_HZ = 4000.0f;
constexpr float RC_FORWARD_STEP_HZ = 200.0f;
constexpr int RC_CH8_ON = 90;
constexpr uint32_t RC_TIMEOUT_MS = 200;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
constexpr float ACCEL_STEPS_PER_S2 = 600.0f;
constexpr uint32_t STEP_PULSE_US = 4;
constexpr uint32_t MIN_STEP_INTERVAL_US = 100;
constexpr size_t LINE_BUFFER_SIZE = 80;
// Explicit opt-in for an isolated bench without the CH7 safety input.
constexpr bool ALLOW_USB_BENCH_MOTION = false;
}

namespace Ultrasonic {
// Match the user's verified sensor firmware: idle HIGH, LOW trigger pulse.
constexpr uint32_t TRIGGER_PULSE_US = 12;
constexpr int STOP_CM = UltrasonicLink::STOP_CM;
constexpr uint32_t INTERVAL_US = 60000;
constexpr uint32_t TIMEOUT_US = 30000;
}

static bool driverEnabled = false;
static bool running = false;
static bool continuous = false;
static uint32_t remainingPulses = 0;
static float targetStepHz = 0.0f;
static float currentStepHz = 0.0f;
static uint32_t lastSpeedUpdateUs = 0;
static uint32_t nextStepUs = 0;
static char lineBuffer[Motor::LINE_BUFFER_SIZE];
static size_t lineLength = 0;

static uint8_t rcWindow[RcControlLink::PACKET_SIZE];
static size_t rcUsed = 0;
static uint32_t lastRcPacketMs = 0;
static uint32_t lastRcLogMs = 0;
static bool rcLinkLive = false;
static bool rcSpinning = false;
static bool usbStopLatched = false;
static bool lastRcEnabled = false;
static int lastRcCh8 = -100;
enum class UltrasonicState : uint8_t { IDLE, WAIT_RISE, WAIT_FALL, READY };
static volatile UltrasonicState ultrasonicState = UltrasonicState::IDLE;
static uint32_t ultrasonicStartedUs = 0;
static volatile uint32_t ultrasonicRiseUs = 0;
static volatile uint32_t ultrasonicFallUs = 0;
static uint32_t ultrasonicLastFinishedUs = 0;
static int ultrasonicDistanceCm = -1;
static bool ultrasonicValid = false;
// Chassis obstacle status; never gates this board's stepper commands.
static bool ultrasonicObstacle = true;
static bool ultrasonicMonitorEnabled = false;
static uint32_t lastUltrasonicLogMs = 0;
static uint32_t distanceTxPackets = 0;
static uint32_t rcRxBytes = 0;

static float pulsesPerRevolution() {
    return static_cast<float>(Motor::FULL_STEPS_PER_REV * Motor::MICROSTEPS);
}

static void setDriverEnabled(bool enabled) {
    driverEnabled = enabled;
    digitalWrite(Pin::ENABLE, enabled ? LOW : HIGH);
}

static void setDirection(bool forward) {
    digitalWrite(Pin::DIR, forward ? Pin::DIR_FORWARD_LEVEL
                                   : !Pin::DIR_FORWARD_LEVEL);
}

static void recordInvalidUltrasonicReading() {
    ultrasonicDistanceCm = -1;
    ultrasonicValid = false;
    ultrasonicObstacle = true;
}

static void finishUltrasonicPing(uint32_t nowUs) {
    ultrasonicLastFinishedUs = nowUs;
    // Send only completed measurements, including invalid results.  A stalled
    // measurement loop cannot keep old distance data alive on the chassis.
    static uint8_t sequence = 0;
    uint8_t packet[UltrasonicLink::PACKET_SIZE];
    UltrasonicLink::encode(packet, ultrasonicValid, ultrasonicDistanceCm, sequence++);
    // Arduino-Pico availableForWrite() returns 0/1, NOT free byte capacity.
    // write() waits for UART FIFO space; 9 bytes at 115200 take <0.8ms.
    if(Serial1.write(packet, sizeof(packet)) == sizeof(packet))
        ++distanceTxPackets;
}

static void captureUltrasonicEcho() {
    const bool high = digitalRead(Pin::ULTRASONIC_ECHO) == HIGH;
    if(high && ultrasonicState == UltrasonicState::WAIT_RISE) {
        ultrasonicRiseUs = micros();
        ultrasonicState = UltrasonicState::WAIT_FALL;
    } else if(!high && ultrasonicState == UltrasonicState::WAIT_FALL) {
        ultrasonicFallUs = micros();
        ultrasonicState = UltrasonicState::READY;
    }
}

static void updateUltrasonic() {
    const uint32_t nowUs = micros();
    if(ultrasonicState == UltrasonicState::IDLE) {
        if(nowUs - ultrasonicLastFinishedUs < Ultrasonic::INTERVAL_US) return;
        ultrasonicStartedUs = micros();
        ultrasonicState = UltrasonicState::WAIT_RISE;
        digitalWrite(Pin::ULTRASONIC_TRIG, LOW);
        delayMicroseconds(Ultrasonic::TRIGGER_PULSE_US);
        digitalWrite(Pin::ULTRASONIC_TRIG, HIGH);
        return;
    }

    if(ultrasonicState != UltrasonicState::READY &&
       nowUs - ultrasonicStartedUs < Ultrasonic::TIMEOUT_US) return;

    // Take a coherent snapshot and disarm. Publish only after restoring IRQs.
    const uint32_t irq = save_and_disable_interrupts();
    const bool ready = ultrasonicState == UltrasonicState::READY;
    const uint32_t fallUs = ultrasonicFallUs;
    const uint32_t widthUs = fallUs - ultrasonicRiseUs;
    ultrasonicState = UltrasonicState::IDLE;
    restore_interrupts(irq);

    const int distanceCm = static_cast<int>(widthUs / 58U);
    if(ready && fallUs - ultrasonicStartedUs <= Ultrasonic::TIMEOUT_US &&
       distanceCm >= 2 && distanceCm <= 400) {
        ultrasonicDistanceCm = distanceCm;
        ultrasonicValid = true;
        ultrasonicObstacle = distanceCm <= Ultrasonic::STOP_CM;
    } else {
        recordInvalidUltrasonicReading();
    }
    finishUltrasonicPing(micros());
}

static void printDriverInfo() {
    Serial.println("TMC2209 standalone STEP/DIR mode (no UART/SPI detection).");
    Serial.println("Expected: MS1=GND, MS2=GND, CLK=GND; 1/8 input microstep.");
    Serial.println("US-100 GP5=TRIG, GP6=ECHO; BLOCK <=173cm / invalid, CLEAR >173cm.");
    Serial.println("Distance UART: GP0 TX -> small_sweet GP5 RX, 115200; chassis forward interlock.");
}

static void stopMotor() {
    running = false;
    continuous = false;
    remainingPulses = 0;
    targetStepHz = 0.0f;
    currentStepHz = 0.0f;
    digitalWrite(Pin::STEP, LOW);
    setDriverEnabled(false);
    Serial.println("STOP: driver disabled");
}

static float clampRpm(float rpm) {
    const float magnitude = fabsf(rpm);
    return magnitude > Motor::MAX_RPM ? Motor::MAX_RPM : magnitude;
}

static void startMotion(long signedPulses, float signedRpm, bool isContinuous) {
    if(signedPulses == 0 && !isContinuous) {
        Serial.println("Move ignored: step pulses must not be zero.");
        return;
    }
    if(signedRpm == 0.0f) {
        Serial.println("Move ignored: RPM must not be zero.");
        return;
    }

    const bool forward = isContinuous ? signedRpm > 0.0f : signedPulses > 0;
    const float rpm = clampRpm(signedRpm);
    setDirection(forward);
    setDriverEnabled(true);
    continuous = isContinuous;
    remainingPulses = isContinuous ? 0U : static_cast<uint32_t>(labs(signedPulses));
    targetStepHz = rpm * pulsesPerRevolution() / 60.0f;
    currentStepHz = 0.0f;
    running = true;
    lastSpeedUpdateUs = micros();
    nextStepUs = lastSpeedUpdateUs;

    Serial.print("RUN: ");
    Serial.print(forward ? "forward" : "reverse");
    Serial.print(", ");
    Serial.print(rpm, 1);
    Serial.print(" RPM (");
    Serial.print(targetStepHz, 1);
    Serial.println(" STEP pulses/s)");
}

static void startPulseRate(float signedStepHz) {
    if(signedStepHz == 0.0f) {
        Serial.println("Pulse rate ignored: pulses/s must not be zero.");
        return;
    }

    const bool forward = signedStepHz > 0.0f;
    const float stepHz = min(fabsf(signedStepHz), Motor::MAX_STEP_HZ);
    setDirection(forward);
    setDriverEnabled(true);
    continuous = true;
    remainingPulses = 0;
    targetStepHz = stepHz;
    currentStepHz = 0.0f;
    running = true;
    lastSpeedUpdateUs = micros();
    nextStepUs = lastSpeedUpdateUs;

    Serial.print("RUN: ");
    Serial.print(forward ? "forward" : "reverse");
    Serial.print(", ");
    Serial.print(stepHz, 0);
    Serial.println(" STEP pulses/s, continuous");
}

static void updateMotor() {
    if(!running)
        return;

    const uint32_t now = micros();
    const float elapsedS = static_cast<float>(now - lastSpeedUpdateUs) / 1000000.0f;
    if(elapsedS > 0.0f) {
        currentStepHz = min(targetStepHz,
                            currentStepHz + Motor::ACCEL_STEPS_PER_S2 * elapsedS);
        lastSpeedUpdateUs = now;
    }
    if(currentStepHz < 1.0f || static_cast<int32_t>(now - nextStepUs) < 0)
        return;

    digitalWrite(Pin::STEP, HIGH);
    delayMicroseconds(Motor::STEP_PULSE_US);
    digitalWrite(Pin::STEP, LOW);

    if(!continuous && --remainingPulses == 0U) {
        stopMotor();
        return;
    }

    const uint32_t intervalUs = max(
        Motor::MIN_STEP_INTERVAL_US,
        static_cast<uint32_t>(1000000.0f / currentStepHz));
    nextStepUs = now + intervalUs;
}

static bool rcAlreadyForward() {
    return running && continuous &&
           digitalRead(Pin::DIR) == Pin::DIR_FORWARD_LEVEL &&
           targetStepHz == Motor::RC_FORWARD_STEP_HZ;
}

static void startRcSpin() {
    if(rcAlreadyForward()) {
        rcSpinning = true;
        return;
    }
    startPulseRate(Motor::RC_FORWARD_STEP_HZ);
    rcSpinning = running;
}

static void stopRcSpin() {
    if(!running && !driverEnabled && !rcSpinning)
        return;
    stopMotor();
    rcSpinning = false;
}

static int rcSignedAxis(uint8_t value) {
    return value <= 127 ? static_cast<int>(value) : static_cast<int>(value) - 256;
}

static uint16_t rcCrc(const uint8_t *data, size_t count) {
    uint16_t crc = 0xFFFF;
    while(count--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for(uint8_t bit = 0; bit < 8; ++bit)
            crc = static_cast<uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}

static void applyRcCommand(bool enabled, int ch8) {
    lastRcEnabled = enabled;
    lastRcCh8 = ch8;
    const bool runRequested = enabled && ch8 >= Motor::RC_CH8_ON;
    if(!runRequested)
        usbStopLatched = false;
    if(runRequested && !usbStopLatched)
        startRcSpin();
    else
        stopRcSpin();
}

static void receiveRcByte(uint8_t value) {
    ++rcRxBytes;
    rcWindow[rcUsed++] = value;
    if(rcUsed < sizeof(rcWindow))
        return;

    const int x = rcSignedAxis(rcWindow[5]);
    const int y = rcSignedAxis(rcWindow[6]);
    const int yaw = rcSignedAxis(rcWindow[7]);
    const int ch10 = rcSignedAxis(rcWindow[8]);
    const int ch7 = rcSignedAxis(rcWindow[9]);
    const int ch9 = rcSignedAxis(rcWindow[10]);
    const int ch8 = rcSignedAxis(rcWindow[11]);
    const uint16_t receivedCrc = static_cast<uint16_t>(rcWindow[14]) |
                                 (static_cast<uint16_t>(rcWindow[15]) << 8);
    const bool valid = rcWindow[0] == 0xA5 && rcWindow[1] == 0x5A &&
                       rcWindow[2] == RcControlLink::VERSION && rcWindow[4] <= 1 &&
                       abs(x) <= 100 && abs(y) <= 100 && abs(yaw) <= 100 &&
                       abs(ch10) <= 100 && abs(ch7) <= 100 && abs(ch9) <= 100 &&
                       abs(ch8) <= 100 && rcCrc(rcWindow + 2, 12) == receivedCrc;
    if(valid) {
        lastRcPacketMs = millis();
        rcLinkLive = true;
        applyRcCommand(rcWindow[4] != 0 && ch7 > RcControlLink::SAFETY_STOP_MAX, ch8);
        rcUsed = 0;
    } else {
        memmove(rcWindow, rcWindow + 1, sizeof(rcWindow) - 1);
        rcUsed = sizeof(rcWindow) - 1;
    }
}

static void updateRcLink() {
    while(Serial1.available() > 0)
        receiveRcByte(static_cast<uint8_t>(Serial1.read()));

    if(rcLinkLive && millis() - lastRcPacketMs > Motor::RC_TIMEOUT_MS) {
        rcLinkLive = false;
        lastRcEnabled = false;
        lastRcCh8 = -100;
        usbStopLatched = false;
        rcUsed = 0;
        while(Serial1.available() > 0)
            Serial1.read();
        stopRcSpin();
        Serial.println("RC timeout: STOP");
    }

    const uint32_t now = millis();
    if(now - lastRcLogMs < 1000)
        return;
    lastRcLogMs = now;
    Serial.print("RC=");
    Serial.print(rcLinkLive ? "OK" : "LOST");
    Serial.print(" enabled=");
    Serial.print(lastRcEnabled ? 1 : 0);
    Serial.print(" CH8=");
    Serial.print(lastRcCh8);
    Serial.print(" spin=");
    Serial.print(rcSpinning ? "FORWARD-200" : "STOP");
    Serial.print(" US100=");
    if(ultrasonicValid) Serial.print(ultrasonicDistanceCm);
    else Serial.print("INVALID");
    Serial.print("cm obstacle=");
    Serial.print(ultrasonicObstacle ? "BLOCK" : "CLEAR");
    Serial.print(" driver=TMC2209-STEPDIR RC_BYTES=");
    Serial.print(rcRxBytes);
    Serial.print(" US_TX=");
    Serial.println(distanceTxPackets);
}

static void printHelp() {
    Serial.println("\nTMC2209 USB test commands (115200 baud, newline):");
    if(Motor::ALLOW_USB_BENCH_MOTION) {
        Serial.println("  m <signed_step_pulses> [rpm]  finite move; e.g. m 1600 10");
        Serial.println("  c <signed_rpm>            continuous; e.g. c -5");
        Serial.println("  p <signed_step_hz>        continuous pulses/s; e.g. p 200");
        Serial.println("  e / d                      enable / disable without moving");
    }
    Serial.println("  s                          immediate stop + disable");
    Serial.println("  i                          print standalone wiring information");
    Serial.println("  u                          toggle live US-100 distance monitor");
    Serial.println("  RC CH8 about +97 and CH7 armed: forward at 200 pulses/s");
    Serial.println("  Mode: MS1=GND, MS2=GND => 1/8 input microstep (1600 pulses/rev)");
}

static char *skipWhitespace(char *text) {
    while(*text == ' ' || *text == '\t')
        ++text;
    return text;
}

static bool parseFloatArgument(char *text, float &value) {
    char *end = nullptr;
    text = skipWhitespace(text);
    value = strtof(text, &end);
    return end != text && *skipWhitespace(end) == '\0';
}

static void handleCommand(char *line) {
    line = skipWhitespace(line);
    if(*line == '\0')
        return;

    if(strcmp(line, "h") == 0 || strcmp(line, "help") == 0) {
        printHelp();
        return;
    }
    if(strcmp(line, "s") == 0 || strcmp(line, "stop") == 0) {
        usbStopLatched = true;
        rcSpinning = false;
        stopMotor();
        return;
    }
    if(strcmp(line, "i") == 0 || strcmp(line, "info") == 0) {
        stopMotor();
        printDriverInfo();
        return;
    }
    if(strcmp(line, "u") == 0) {
        ultrasonicMonitorEnabled = !ultrasonicMonitorEnabled;
        Serial.print("US-100 live monitor: ");
        Serial.println(ultrasonicMonitorEnabled ? "ON" : "OFF");
        return;
    }
    if(rcLinkLive || !Motor::ALLOW_USB_BENCH_MOTION) {
        Serial.println("CH7 safety controls motion. USB s stops the driver.");
        return;
    }
    if(strcmp(line, "e") == 0) {
        setDriverEnabled(true);
        Serial.println("Driver enabled; no STEP pulses are being sent.");
        return;
    }
    if(strcmp(line, "d") == 0) {
        stopMotor();
        return;
    }

    char *argument = line + 1;
    if(line[0] == 'm' && (line[1] == ' ' || line[1] == '\t')) {
        char *end = nullptr;
        argument = skipWhitespace(argument);
        const long pulses = strtol(argument, &end, 10);
        if(end == argument) {
            Serial.println("Move ignored: expected a signed STEP pulse count.");
            return;
        }
        float rpm = 10.0f;
        char *rpmText = skipWhitespace(end);
        if(*rpmText != '\0' && !parseFloatArgument(rpmText, rpm)) {
            Serial.println("Move ignored: RPM must be a number.");
            return;
        }
        startMotion(pulses, rpm, false);
        return;
    }

    float value = 0.0f;
    if(line[0] == 'c' && (line[1] == ' ' || line[1] == '\t') &&
       parseFloatArgument(argument, value)) {
        startMotion(0, value, true);
        return;
    }
    if(line[0] == 'p' && (line[1] == ' ' || line[1] == '\t') &&
       parseFloatArgument(argument, value)) {
        startPulseRate(value);
        return;
    }
    if(line[0] == 'c' || line[0] == 'p') {
        Serial.println("Command ignored: expected a signed number after the command.");
        return;
    }
    Serial.println("Unknown command. Type h for help.");
}

static void pollSerial() {
    while(Serial.available() > 0) {
        const char character = static_cast<char>(Serial.read());
        if(character == '\r' || character == '\n') {
            if(lineLength > 0) {
                lineBuffer[lineLength] = '\0';
                handleCommand(lineBuffer);
                lineLength = 0;
            }
        } else if(lineLength < sizeof(lineBuffer) - 1U) {
            lineBuffer[lineLength++] = character;
        } else {
            lineLength = 0;
            Serial.println("Command too long; discarded.");
        }
    }
}

void setup() {
    pinMode(Pin::STEP, OUTPUT);
    pinMode(Pin::DIR, OUTPUT);
    pinMode(Pin::ENABLE, OUTPUT);
    pinMode(Pin::ULTRASONIC_TRIG, OUTPUT);
    pinMode(Pin::ULTRASONIC_ECHO, INPUT_PULLDOWN);
    attachInterrupt(Pin::ULTRASONIC_ECHO, captureUltrasonicEcho, CHANGE);
    digitalWrite(Pin::STEP, LOW);
    setDirection(true);
    digitalWrite(Pin::ULTRASONIC_TRIG, HIGH);
    setDriverEnabled(false);

    Serial.begin(115200);
    delay(250);
    Serial.println("TMC2209 stepper starting. Driver starts disabled.");
    printDriverInfo();

    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.setFIFOSize(256);
    Serial1.begin(115200);
    Serial.println("RC UART GP1 RX at 115200. CH8 +97 = 200 pulses/s forward.");
    Serial.println("US-100 sent on GP0 TX to chassis; CH8 stepper control is independent.");
    printHelp();
    watchdog_enable(Motor::WATCHDOG_TIMEOUT_MS, true);
}

void loop() {
    updateUltrasonic();
    updateRcLink();
    pollSerial();
    updateMotor();
    if(ultrasonicMonitorEnabled && millis() - lastUltrasonicLogMs >= 100) {
        lastUltrasonicLogMs = millis();
        Serial.print("US-100: DIST=");
        Serial.print(ultrasonicDistanceCm);
        Serial.print("cm valid=");
        Serial.print(ultrasonicValid ? 1 : 0);
        Serial.print(" obstacle=");
        Serial.println(ultrasonicObstacle ? "BLOCK" : "CLEAR");
    }
    watchdog_update();
}
