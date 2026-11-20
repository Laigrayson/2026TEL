// Reuse the working main.txt pin map, direction corrections and USB commands.
// The old ESC main.cpp is excluded by the four_motor_rc build environment.
#define setup originalMotorSetup
#define loop originalMotorLoop
#include "../main.txt"
#undef setup
#undef loop
#include <cstring>
#include <Servo.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <pico/time.h>
#include "../../../common/ultrasonic_link.h"
#include "../../../common/rc_control_link.h"

// Independent UART1: stepper GP0 TX -> this board GP5 RX.
// GP4 is the unused TX of this UART; bridge RC still arrives on GP1.
static UltrasonicLink::Receiver distanceLink;
static int requestedRadioX = 0;
static int requestedRadioY = 0;
static int requestedRadioYaw = 0;
static uint32_t distanceRxBytes = 0;
static uint32_t rcRxBytes = 0;

static constexpr uint32_t LINK_TIMEOUT_MS = 200;
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
// Radio axis directions, adjusted from the first driving test.
// CH1 and CH4 were both physically reversed; CH2 was already correct.
static constexpr int STRAFE_SIGN = -1;
static constexpr int FORWARD_SIGN = 1;
static constexpr int YAW_SIGN = -1;

// 5 顆 TT 閘門馬達（3 片雙路 L9110S），直接接在馬達 Pico。
static constexpr uint8_t PIN_TT_IA[5] = {16, 18, 20, 22, 27};
static constexpr uint8_t PIN_TT_IB[5] = {17, 19, 21, 26, 28};

// ============================================================================
// L9110S / TT 儲球閘門狀態機
//
// CH9 每次 LOW→HIGH 只執行一次：
//   當前 TT 開啟 1.5 秒 → 立即反向返回 1.5 秒 → 切換下一顆。
// 順序 TT1→TT2→TT3→TT4→TT5→TT1，動作期間忽略新的按鍵。
// ============================================================================
namespace TtGateManager {
static constexpr uint8_t MOTOR_COUNT = 5;
// 無編碼器 TT 馬達無法量測精確角度，使用時間控制並依實機校正。
static constexpr uint32_t GATE_MOVE_MS = 1500;
static constexpr uint32_t BALL_DROP_WAIT_MS = 0;

enum class State : uint8_t {
    IDLE,
    OPENING,
    WAITING_FOR_BALL,
    CLOSING,
};

static State state = State::IDLE;
static uint8_t activeMotor = 0;
static bool lastTrigger = false;
static uint64_t stateStartedUs = 0;

static void stopMotor(uint8_t motor) {
    if(motor >= MOTOR_COUNT)
        return;
    gpio_put(PIN_TT_IA[motor], 0);
    gpio_put(PIN_TT_IB[motor], 0);
}

static void stopAll() {
    for(uint8_t motor = 0; motor < MOTOR_COUNT; motor++)
        stopMotor(motor);
}

static void driveForward(uint8_t motor) {
    stopAll();
    gpio_put(PIN_TT_IA[motor], 0);
    gpio_put(PIN_TT_IB[motor], 1);
}

static void driveReverse(uint8_t motor) {
    stopAll();
    gpio_put(PIN_TT_IA[motor], 1);
    gpio_put(PIN_TT_IB[motor], 0);
}

static void init() {
    for(uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        gpio_init(PIN_TT_IA[motor]);
        gpio_set_dir(PIN_TT_IA[motor], GPIO_OUT);
        gpio_init(PIN_TT_IB[motor]);
        gpio_set_dir(PIN_TT_IB[motor], GPIO_OUT);
    }
    stopAll();
}

static void setTrigger(bool trigger) {
    bool risingEdge = trigger && !lastTrigger;
    lastTrigger = trigger;

    if(!risingEdge || state != State::IDLE)
        return;

    driveForward(activeMotor);
    stateStartedUs = time_us_64();
    state = State::OPENING;
}

static void update() {
    uint64_t now = time_us_64();
    uint64_t elapsedUs = now - stateStartedUs;

    switch(state) {
    case State::OPENING:
        if(elapsedUs >= static_cast<uint64_t>(GATE_MOVE_MS) * 1000ULL) {
            stopMotor(activeMotor);
            stateStartedUs = now;
            state = State::WAITING_FOR_BALL;
        }
        break;

    case State::WAITING_FOR_BALL:
        if(elapsedUs >= static_cast<uint64_t>(BALL_DROP_WAIT_MS) * 1000ULL) {
            driveReverse(activeMotor);
            stateStartedUs = now;
            state = State::CLOSING;
        }
        break;

    case State::CLOSING:
        if(elapsedUs >= static_cast<uint64_t>(GATE_MOVE_MS) * 1000ULL) {
            stopMotor(activeMotor);
            activeMotor++;
            if(activeMotor >= MOTOR_COUNT)
                activeMotor = 0;
            state = State::IDLE;
        }
        break;

    case State::IDLE:
    default:
        break;
    }
}

static void estop() {
    stopAll();
    state = State::IDLE;
    // 若斷線時按鈕仍按住，必須先放開再按才可重新觸發。
    lastTrigger = true;
}
} // namespace TtGateManager

// 24 V drive tuning: deliberately calm response.  A dead zone removes receiver
// centre jitter, strong expo softens small stick movements, and the low ramp
// rates prevent the high-voltage motors from jumping or overshooting.
// Increase REMOTE_MAX_PWM later only after the chassis tracks smoothly.
static constexpr int REMOTE_DEADZONE = 15;
static constexpr int REMOTE_START_PWM = 28;
static constexpr int REMOTE_MAX_PWM = 80;
static constexpr int REMOTE_EXPO_PERCENT = 85;
static constexpr int REMOTE_ACCEL_STEP = 1;
static constexpr int REMOTE_DECEL_STEP = 2;
static int smoothedX = 0;
static int smoothedY = 0;
static int smoothedYaw = 0;

// Forward veered left while reverse tracked correctly.  That means the right
// side was effectively stronger only in the forward direction, so trim only
// positive (forward wheel rotation) commands on that side.  Reverse stays 100%.
// Raise/lower 90 in small steps later if the chassis still does not track straight.
static constexpr int FL_FORWARD_GAIN = 100;
static constexpr int FR_FORWARD_GAIN = 90;
static constexpr int BL_FORWARD_GAIN = 100;
static constexpr int BR_FORWARD_GAIN = 90;
static constexpr int FL_REVERSE_GAIN = 100;
static constexpr int FR_REVERSE_GAIN = 100;
static constexpr int BL_REVERSE_GAIN = 100;
static constexpr int BR_REVERSE_GAIN = 100;

// Hobbywing Skywalker 40A V2 throttle output on the same motor Pico.
// White signal wire -> GP14; black wire -> GND (never a GPIO).
static constexpr uint8_t BRUSHLESS_ESC_PIN = 14;
static constexpr int BRUSHLESS_STOP_US = 1100;
static constexpr int BRUSHLESS_FULL_US = 1465;
static Servo brushlessEsc;
static bool sawBrushlessThrottleLow = false;
static bool brushlessEnabled = false;
static int lastCh5 = -100;
static int brushlessUs = BRUSHLESS_STOP_US;

static constexpr uint8_t POSITION_SERVO_PIN = 15;
static constexpr int POSITION_SERVO_MIN_US = 600;
static constexpr int POSITION_SERVO_MAX_US = 1500;
static constexpr int POSITION_SERVO_STEP_US = 12;
static constexpr uint32_t SERVO_UPDATE_MS = 20;
static constexpr int CH10_PRESET_ENTER = -95;
static constexpr int CH10_PRESET_EXIT = -90;
static Servo positionServo;
static int lastCh6 = 0;
static int lastCh7 = -100;
static int lastCh10 = -100;
static int positionServoUs = 1020;
static bool useServoPreset = true;
static uint32_t lastServoUpdateMs = 0;

static void forceSafeStartupOutputs() {
    // Establish harmless levels before USB startup delays or peripheral setup.
    const uint8_t safeLowPins[] = {
        MOTOR_FR_PWM, MOTOR_FR_DIR, MOTOR_FL_PWM, MOTOR_FL_DIR,
        MOTOR_BR_PWM, MOTOR_BR_DIR, MOTOR_BL_PWM, MOTOR_BL_DIR,
        BRUSHLESS_ESC_PIN, POSITION_SERVO_PIN,
    };
    for (uint8_t pin : safeLowPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    for (uint8_t motor = 0; motor < 5; ++motor) {
        gpio_init(PIN_TT_IA[motor]);
        gpio_set_dir(PIN_TT_IA[motor], GPIO_OUT);
        gpio_put(PIN_TT_IA[motor], 0);
        gpio_init(PIN_TT_IB[motor]);
        gpio_set_dir(PIN_TT_IB[motor], GPIO_OUT);
        gpio_put(PIN_TT_IB[motor], 0);
    }
}

static uint8_t packetWindow[RcControlLink::PACKET_SIZE];
static size_t packetUsed = 0;
static uint32_t lastPacketMs = 0;
static bool havePacket = false;
static bool timedOut = false;
static bool usbStopLatched = false;
static bool linkEnabled = false;
static int lastX = 0;
static int lastY = 0;
static int lastYaw = 0;
static int lastCh9 = -100;
static int lastCh8 = -100;
static int lastFL = 0;
static int lastFR = 0;
static int lastBL = 0;
static int lastBR = 0;
static uint32_t lastDebugMs = 0;

static uint16_t linkCrc(const uint8_t* data, size_t count) {
    uint16_t crc = 0xFFFF;
    while (count--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = static_cast<uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}

static int signedAxis(uint8_t value) {
    return value <= 127 ? static_cast<int>(value) : static_cast<int>(value) - 256;
}

static void stopBrushless() {
    brushlessEnabled = false;
    brushlessUs = BRUSHLESS_STOP_US;
    brushlessEsc.writeMicroseconds(BRUSHLESS_STOP_US);
}

static void applyBrushless(int ch5) {
    if (ch5 <= -90) sawBrushlessThrottleLow = true;
    if (!sawBrushlessThrottleLow) {
        stopBrushless();
        return;
    }
    brushlessEnabled = true;
    brushlessUs = ch5 >= 90 ? BRUSHLESS_FULL_US : BRUSHLESS_STOP_US;
    brushlessEsc.writeMicroseconds(brushlessUs);
}

static void updatePositionServo() {
    if (!linkEnabled || usbStopLatched) {
        useServoPreset = true;
        if (positionServo.attached()) {
            positionServo.detach();
            digitalWrite(POSITION_SERVO_PIN, LOW);
        }
        return;
    }
    const uint32_t now = millis();
    if (now - lastServoUpdateMs < SERVO_UPDATE_MS) return;
    lastServoUpdateMs = now;

    const int baseUs = lastCh6 >= 50 ? 840 : (lastCh6 <= -50 ? 1200 : 1020);
    const int knob = constrain(lastCh10, -100, 55);
    // The idle detent includes the observed radio minimum (-97).
    if (knob <= CH10_PRESET_ENTER) useServoPreset = true;
    else if (knob >= CH10_PRESET_EXIT) useServoPreset = false;

    int targetUs = baseUs;
    if (!useServoPreset) {
        // Reserve a separate lower trim point so the knob can reach 600us
        // outside the preset detent, while preserving CH6's position at zero.
        const int trim = max(knob, CH10_PRESET_EXIT);
        if (trim < 0) {
            const int travel = -CH10_PRESET_EXIT;
            targetUs = baseUs - ((baseUs - POSITION_SERVO_MIN_US) * -trim + travel / 2) / travel;
        } else {
            targetUs = baseUs + ((POSITION_SERVO_MAX_US - baseUs) * trim + 27) / 55;
        }
    }
    positionServoUs += constrain(targetUs - positionServoUs,
                                 -POSITION_SERVO_STEP_US, POSITION_SERVO_STEP_US);
    if (positionServo.attached()) {
        positionServo.writeMicroseconds(positionServoUs);
    } else {
        positionServo.attach(POSITION_SERVO_PIN, POSITION_SERVO_MIN_US,
                             POSITION_SERVO_MAX_US, positionServoUs);
    }
}

static bool forwardBlocked() {
    return lastCh7 < RcControlLink::BYPASS_MIN && distanceLink.blocked;
}

static int wheelPwm(int normalized, int forwardGain, int reverseGain) {
    if (normalized == 0) return 0;
    const int magnitude = constrain(abs(normalized), 0, 100);
    int pwm = REMOTE_START_PWM +
              magnitude * (REMOTE_MAX_PWM - REMOTE_START_PWM) / 100;
    pwm = pwm * (normalized > 0 ? forwardGain : reverseGain) / 100;
    return normalized > 0 ? pwm : -pwm;
}

static int shapeRemoteAxis(int value) {
    const int rawMagnitude = constrain(abs(value), 0, 100);
    if (rawMagnitude <= REMOTE_DEADZONE) return 0;

    // Rescale after the dead zone so full stick travel still reaches 100%, then
    // apply strong quadratic expo for fine low-speed control.
    const int magnitude = (rawMagnitude - REMOTE_DEADZONE) * 100 /
                          (100 - REMOTE_DEADZONE);
    const int shapedMagnitude = magnitude *
        ((100 - REMOTE_EXPO_PERCENT) * 100 +
         REMOTE_EXPO_PERCENT * magnitude) / 10000;
    return value < 0 ? -shapedMagnitude : shapedMagnitude;
}

static int approachRemoteAxis(int current, int target) {
    const bool slowingDown =
        (current > 0 && target >= 0 && target < current) ||
        (current < 0 && target <= 0 && target > current) ||
        (current != 0 && target == 0) ||
        (current > 0 && target < 0) ||
        (current < 0 && target > 0);
    const int step = slowingDown ? REMOTE_DECEL_STEP : REMOTE_ACCEL_STEP;
    if (current < target) return min(current + step, target);
    if (current > target) return max(current - step, target);
    return current;
}

static void resetRemoteSmoothing() {
    smoothedX = 0;
    smoothedY = 0;
    smoothedYaw = 0;
}

static void applyRemote(int radioX, int radioY, int radioYaw) {
    requestedRadioX = radioX;
    requestedRadioY = radioY;
    requestedRadioYaw = radioYaw;
    // Standard X-roller mecanum mix:
    //   right stick X = strafe, right stick Y = forward/back,
    //   left stick X = rotation. Left yaw takes priority as a stationary turn.
    int targetX = shapeRemoteAxis(radioX * STRAFE_SIGN);
    int targetY = shapeRemoteAxis(radioY * FORWARD_SIGN);
    const int targetYaw = shapeRemoteAxis(radioYaw * YAW_SIGN);

    if (targetYaw > 0) {
        // Drop both the current stick translation and any residual ramp from
        // the preceding drive command before calculating the left turn.
        targetX = 0;
        targetY = 0;
        smoothedX = 0;
        smoothedY = 0;
    }

    // Clamp just the forward component; retain retreat, strafe and rotation.
    // Bypass the normal deceleration ramp when an obstacle/link fault occurs.
    const bool blockForward = forwardBlocked();
    if(blockForward) {
        if(targetY > 0) targetY = 0;
        if(smoothedY > 0) smoothedY = 0;
    }

    smoothedX = approachRemoteAxis(smoothedX, targetX);
    smoothedY = approachRemoteAxis(smoothedY, targetY);
    smoothedYaw = approachRemoteAxis(smoothedYaw, targetYaw);
    const int x = smoothedX;
    const int y = smoothedY;
    const int yaw = smoothedYaw;

    // Front and rear axle polarity is opposite for longitudinal travel on
    // this chassis. Preserve the strafe/yaw contributions verified on the car.
    int fl =  y - x - yaw;
    int fr =  y + x + yaw;
    int bl = -y + x - yaw;
    int br = -y - x + yaw;

    // Preserve the requested direction while scaling combined commands so no
    // wheel exceeds 100% of the requested range.
    const int peak = max(100, max(max(abs(fl), abs(fr)), max(abs(bl), abs(br))));
    fl = fl * 100 / peak;
    fr = fr * 100 / peak;
    bl = bl * 100 / peak;
    br = br * 100 / peak;

    lastX = x;
    lastY = y;
    lastYaw = yaw;
    // Keep straight-drive trim out of strafe and rotation. For rear-right,
    // physical forward uses a negative signed command on this chassis.
    const bool balanceOpposingWheels = y == 0;
    lastFL = wheelPwm(fl, FL_FORWARD_GAIN, FL_REVERSE_GAIN);
    lastFR = wheelPwm(fr, balanceOpposingWheels ? FR_REVERSE_GAIN : FR_FORWARD_GAIN, FR_REVERSE_GAIN);
    lastBL = wheelPwm(bl, BL_FORWARD_GAIN, BL_REVERSE_GAIN);
    lastBR = wheelPwm(br, BR_REVERSE_GAIN, balanceOpposingWheels ? BR_REVERSE_GAIN : BR_FORWARD_GAIN);

    motorWrite(motorFL, lastFL);
    motorWrite(motorFR, lastFR);
    motorWrite(motorBL, lastBL);
    motorWrite(motorBR, lastBR);
}

static void receiveLinkByte(uint8_t value) {
    ++rcRxBytes;
    packetWindow[packetUsed++] = value;
    if (packetUsed < sizeof(packetWindow)) return;
    const int x = signedAxis(packetWindow[5]);
    const int y = signedAxis(packetWindow[6]);
    const int yaw = signedAxis(packetWindow[7]);
    const int ch10 = signedAxis(packetWindow[8]);
    const int ch7 = signedAxis(packetWindow[9]);
    const int ch9 = signedAxis(packetWindow[10]);
    const int ch8 = signedAxis(packetWindow[11]);
    const int ch5 = signedAxis(packetWindow[12]);
    const int ch6 = signedAxis(packetWindow[13]);
    const uint16_t receivedCrc = static_cast<uint16_t>(packetWindow[14]) |
                                (static_cast<uint16_t>(packetWindow[15]) << 8);
    const bool valid = packetWindow[0] == 0xA5 && packetWindow[1] == 0x5A &&
        packetWindow[2] == RcControlLink::VERSION && packetWindow[4] <= 1 &&
        abs(x) <= 100 && abs(y) <= 100 && abs(yaw) <= 100 &&
        abs(ch10) <= 100 && abs(ch7) <= 100 && abs(ch9) <= 100 && abs(ch8) <= 100 &&
        abs(ch5) <= 100 && abs(ch6) <= 100 &&
        linkCrc(packetWindow + 2, 12) == receivedCrc;
    if (valid) {
        lastPacketMs = millis();
        havePacket = true;
        timedOut = false;
        linkEnabled = packetWindow[4] != 0 && ch7 > RcControlLink::SAFETY_STOP_MAX;
        lastCh5 = ch5;
        lastCh6 = ch6;
        lastCh7 = ch7;
        lastCh10 = ch10;
        lastCh9 = ch9;
        lastCh8 = ch8; // Stepper Pico consumes CH8; this board only reports it.
        // CH7 is the master safety for CH9 as well. E-stop forces a
        // release-before-repress sequence, so arming cannot start a gate
        // while the CH9 button is already held.
        if (linkEnabled && !usbStopLatched)
            TtGateManager::setTrigger(ch9 > 50);
        else
            TtGateManager::estop();
        if (!linkEnabled) {
            usbStopLatched = false;
            stopAll();
            resetRemoteSmoothing();
            stopBrushless();
            sawBrushlessThrottleLow = ch5 <= -90;
            lastX = lastY = lastYaw = 0;
            lastFL = lastFR = lastBL = lastBR = 0;
        } else if (!usbStopLatched) {
            applyRemote(x, y, yaw);
            applyBrushless(ch5);
        } else {
            stopAll();
            resetRemoteSmoothing();
            stopBrushless();
        }
        packetUsed = 0;
    } else {
        memmove(packetWindow, packetWindow + 1, sizeof(packetWindow) - 1);
        packetUsed = sizeof(packetWindow) - 1;
    }
}

extern "C" void setup() {
    const bool rebootedByWatchdog = watchdog_caused_reboot();
    forceSafeStartupOutputs();
    analogWriteFreq(PWM_FREQ);
    analogWriteRange(PWM_MAX);
    stopAll();
    Serial.begin(115200);
    brushlessEsc.attach(BRUSHLESS_ESC_PIN, 1000, 2000, BRUSHLESS_STOP_US);
    stopBrushless();
    TtGateManager::init();
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.setFIFOSize(256);
    Serial1.begin(115200);
    Serial2.setTX(4);
    Serial2.setRX(5);
    Serial2.setFIFOSize(256);
    Serial2.begin(UltrasonicLink::BAUD);
    Serial.println("Distance UART GP5 RX: CH7 center blocks forward at <=173cm, invalid or timeout.");
    Serial.println("four_motor_rc: full mecanum mix, UART GP0/GP1 at 115200.");
    Serial.printf("CH1/CH4 inverted; PWM=%d..%d, deadzone=%d, expo=%d, accel=%d, decel=%d.\n",
                  REMOTE_START_PWM, REMOTE_MAX_PWM, REMOTE_DEADZONE,
                  REMOTE_EXPO_PERCENT, REMOTE_ACCEL_STEP, REMOTE_DECEL_STEP);
    Serial.println("CH5 ESC GP14: OFF=1100us, ON=1465us.");
    Serial.println("Servo GP15: CH6 selects 1200/1020/840us, CH10 trims around the preset.");
    Serial.println("CH7: down=STOP, center=AVOID, up=BYPASS. CH9 gate sequence unchanged.");
    Serial.println("CH10: minimum detent=CH6 preset; trim -90/0/+55=600/preset/1500us.");
    Serial.printf("Watchdog: 1000ms, previous reboot=%s.\n",
                  rebootedByWatchdog ? "WATCHDOG" : "normal");
    Serial.println("DBR4/CH7 controls motion; USB x/space stops.");
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
}

extern "C" void loop() {
    distanceLink.expire(millis());
    while(Serial2.available()) {
        ++distanceRxBytes;
        distanceLink.feed(static_cast<uint8_t>(Serial2.read()), millis());
    }
    // If a link stalls, discard its buffered commands before processing recovery.
    if (havePacket && millis() - lastPacketMs > LINK_TIMEOUT_MS && !timedOut) {
        stopAll();
        resetRemoteSmoothing();
        stopBrushless();
        TtGateManager::estop();
        timedOut = true;
        linkEnabled = false;
        sawBrushlessThrottleLow = false;
        lastCh7 = -100;
        lastX = lastY = lastYaw = 0;
        lastFL = lastFR = lastBL = lastBR = 0;
        lastCh8 = -100;
        packetUsed = 0;
        while (Serial1.available()) Serial1.read();
        Serial.println("UART timeout: STOP");
    }
    if(forwardBlocked() && linkEnabled && !usbStopLatched && smoothedY > 0)
        applyRemote(requestedRadioX, requestedRadioY, requestedRadioYaw);
    while (Serial1.available()) receiveLinkByte(static_cast<uint8_t>(Serial1.read()));
    TtGateManager::update();
    while (Serial.available()) {
        const char key = static_cast<char>(Serial.read());
        if (key == '\r' || key == '\n') continue;
        // USB stop is latched until bridge CH7 has been turned off.
        if (key == 'x' || key == 'X' || key == ' ') {
            stopAll();
            resetRemoteSmoothing();
            stopBrushless();
            sawBrushlessThrottleLow = false;
            TtGateManager::estop();
            usbStopLatched = true;
            continue;
        }
        if (key == 'h' || key == 'H' || key == '?')
            Serial.println("Motion is controlled by DBR4/CH7. USB x/space stops.");
    }
    updatePositionServo();
    if (millis() - lastDebugMs >= 500) {
        lastDebugMs = millis();
        const bool rxFresh = havePacket && millis() - lastPacketMs <= LINK_TIMEOUT_MS;
        Serial.printf("RX=%d enabled=%d CH2=%d x=%d y=%d yaw=%d | FL=%d FR=%d BL=%d BR=%d | CH5=%d ESC=%dus armed=%d | CH6=%d | CH7=%d MODE=%s | CH8=%d CH9=%d | CH10=%d SERVO=%dus PWM=%s | US_RX=%d DIST=%dcm valid=%d obstacle=%s | RC_BYTES=%lu US_BYTES=%lu\n",
                      rxFresh, linkEnabled, requestedRadioY, lastX, lastY, lastYaw,
                      lastFL, lastFR, lastBL, lastBR,
                      lastCh5, brushlessUs, brushlessEnabled, lastCh6,
                      lastCh7, (!linkEnabled || usbStopLatched) ? "STOP" :
                          (lastCh7 >= RcControlLink::BYPASS_MIN ? "BYPASS" : "AVOID"),
                      lastCh8, lastCh9, lastCh10, positionServoUs,
                      positionServo.attached() ? "ON" : "OFF",
                      distanceLink.fresh(millis()), distanceLink.distanceCm,
                      distanceLink.valid, forwardBlocked() ? "BLOCK" : "CLEAR",
                      static_cast<unsigned long>(rcRxBytes),
                      static_cast<unsigned long>(distanceRxBytes));
    }
    watchdog_update();
}
