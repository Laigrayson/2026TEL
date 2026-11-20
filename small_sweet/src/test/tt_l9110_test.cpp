// Temporary USB bench test for one TT motor on an L9110S.
// Independent of four_motor_rc. Flash env tt_l9110_test only.
//
// Five TT motors on L9110S, same pins and 1500 ms each way as four_motor_rc:
//   TT1 GP16/17, TT2 GP18/19, TT3 GP20/21, TT4 GP22/26, TT5 GP27/28.
// L n runs that motor forward 1.5 s, then reverse 1.5 s, then stop.
// f/r still jog TT1 only. Pico GND must be common with every L9110.
// GP15 -> DS3240 signal. Same endpoints as four_motor_rc: 600..2400 us.
// GP14 -> Hobbywing SkyWalker throttle. Stop 1100 us, full 1940 us.
// Four encoder drive motors, same open-loop map as main.txt. Positive PWM
// is physical forward; the right side DIR is mirrored.
//   FR GP6/GP7, FL GP8/GP9, BR GP10/GP11, BL GP12/GP13.
//
// USB 115200, newline:
//   f          TT1 forward until s
//   r          TT1 reverse until s
//   L 1        TT1 gate move, also L 2 .. L 5
//   s          stop TT, four drive motors, and the brushless ESC
//   v          servo to center, 1500 us
//   v 600      servo to 600 us
//   v 2400     servo to 2400 us
//   b          brushless stop, 1100 us
//   b 1500     brushless throttle pulse
//   w          all four wheels forward at PWM 80
//   w -80      all four wheels reverse
//   1 80       front-right only; 2 left-front, 3 right-rear, 4 left-rear
//   h          help

#include <Arduino.h>
#include <Servo.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <cstdlib>
#include <cstring>

static constexpr uint8_t TT_IA[5] = {16, 18, 20, 22, 27};
static constexpr uint8_t TT_IB[5] = {17, 19, 21, 26, 28};
static constexpr uint32_t TT_MOVE_MS = 1500;
enum class TtPhase : uint8_t { Idle, Opening, Closing };
static constexpr uint8_t PIN_SERVO = 15;
static constexpr int SERVO_MIN_US = 600;
static constexpr int SERVO_CENTER_US = 1500;
static constexpr int SERVO_MAX_US = 2400;
static constexpr uint8_t PIN_ESC = 14;
static constexpr int ESC_STOP_US = 1100;
static constexpr int ESC_FULL_US = 1940;
static constexpr int WHEEL_PWM_MAX = 255;
static constexpr int WHEEL_DEFAULT_PWM = 150;
static constexpr uint32_t WHEEL_PWM_FREQ = 20000;
static constexpr size_t LINE_BUFFER_SIZE = 32;

struct Wheel {
    uint8_t pwmPin;
    uint8_t dirPin;
    bool reversed;
    const char *name;
};

static const Wheel wheels[] = {
    {6, 7, true, "FR"},
    {8, 9, false, "FL"},
    {10, 11, true, "BR"},
    {12, 13, false, "BL"},
};

static char lineBuffer[LINE_BUFFER_SIZE];
static size_t lineLength = 0;
static uint32_t stopAtMs = 0;
static bool timedRun = false;
static TtPhase ttPhase = TtPhase::Idle;
static int ttIndex = 0;
static uint32_t ttPhaseEndsMs = 0;
static Servo positionServo;
static int servoUs = SERVO_CENTER_US;
static Servo brushlessEsc;
static int escUs = ESC_STOP_US;

static void writeEsc(int pulseUs) {
    escUs = constrain(pulseUs, ESC_STOP_US, ESC_FULL_US);
    brushlessEsc.writeMicroseconds(escUs);
    Serial.print("ESC ");
    Serial.print(escUs);
    Serial.println(" us");
}

static void writeWheel(const Wheel &wheel, int speed) {
    speed = constrain(speed, -WHEEL_PWM_MAX, WHEEL_PWM_MAX);
    bool forward = speed >= 0;
    if (wheel.reversed)
        forward = !forward;

    // PWM and DIR are the two channels of one RP2040 PWM slice.
    // analogWrite() initializes that whole slice, so put the PWM pin back
    // into PWM mode and force DIR back to a plain GPIO afterwards.
    analogWrite(wheel.pwmPin, abs(speed));
    gpio_set_function(wheel.dirPin, GPIO_FUNC_SIO);
    gpio_set_dir(wheel.dirPin, GPIO_OUT);
    gpio_put(wheel.dirPin, forward ? 1 : 0);
    gpio_set_function(wheel.pwmPin, GPIO_FUNC_PWM);
}

static void stopWheels() {
    for (const Wheel &wheel : wheels)
        writeWheel(wheel, 0);
}

static void reportWheels(int onlyIndex, int speed) {
    Serial.print("WHEELS");
    for (int index = 0; index < 4; ++index) {
        const int command = (onlyIndex < 0 || onlyIndex == index) ? speed : 0;
        Serial.print(' ');
        Serial.print(wheels[index].name);
        Serial.print('=');
        Serial.print(command);
    }
    Serial.println();
}

static bool wheelSignalOk(const Wheel &wheel, int speed) {
    const uint slice = pwm_gpio_to_slice_num(wheel.pwmPin);
    const uint channel = pwm_gpio_to_channel(wheel.pwmPin);
    const bool pwmMode = gpio_get_function(wheel.pwmPin) == GPIO_FUNC_PWM;
    const bool dirMode = gpio_get_function(wheel.dirPin) == GPIO_FUNC_SIO;
    const bool enabled = (pwm_hw->slice[slice].csr & PWM_CH0_CSR_EN_BITS) != 0;
    const uint32_t compare = pwm_hw->slice[slice].cc;
    const uint16_t level = static_cast<uint16_t>((compare >> (channel == PWM_CHAN_B ? 16 : 0)) & 0xffffu);
    const uint16_t countBefore = static_cast<uint16_t>(pwm_hw->slice[slice].ctr);
    delayMicroseconds(80);
    const uint16_t countAfter = static_cast<uint16_t>(pwm_hw->slice[slice].ctr);
    const bool counting = enabled && countBefore != countAfter;
    const int dirLevel = gpio_get(wheel.dirPin) ? 1 : 0;
    bool forward = speed >= 0;
    if (wheel.reversed)
        forward = !forward;
    const int expectedDir = forward ? 1 : 0;
    const bool levelOk = speed == 0 ? level == 0 : level > 0;
    const bool pwmOk = pwmMode && enabled && levelOk && (speed == 0 || counting);
    const bool dirOk = dirMode && dirLevel == expectedDir;

    Serial.print(wheel.name);
    Serial.print(pwmOk && dirOk ? " 訊號=正常" : " 訊號=異常");
    Serial.print(" PWM腳=GP");
    Serial.print(wheel.pwmPin);
    Serial.print(pwmMode ? " 模式=PWM" : " 模式=不是PWM");
    Serial.print(" 脈寬=");
    Serial.print(level);
    Serial.print(counting ? " 計數=跑" : " 計數=停");
    Serial.print(" DIR腳=GP");
    Serial.print(wheel.dirPin);
    Serial.print(" 讀回=");
    Serial.print(dirLevel);
    Serial.print(" 期望=");
    Serial.println(expectedDir);
    return pwmOk && dirOk;
}

static void reportSignal(int onlyIndex, int speed) {
    int failed = 0;
    bool dirForced = false;
    for (int index = 0; index < 4; ++index) {
        const int command = (onlyIndex < 0 || onlyIndex == index) ? speed : 0;
        const Wheel &wheel = wheels[index];
        if (!wheelSignalOk(wheel, command)) {
            ++failed;
            bool forward = command >= 0;
            if (wheel.reversed)
                forward = !forward;
            const int expectedDir = forward ? 1 : 0;
            const int dirLevel = gpio_get(wheel.dirPin) ? 1 : 0;
            if (gpio_get_function(wheel.dirPin) == GPIO_FUNC_SIO && dirLevel != expectedDir)
                dirForced = true;
        }
    }
    if (failed == 0 && speed != 0) {
        Serial.println("檢查: 訊號正常。Pico 的 PWM/DIR 有送出。輪子不轉就是馬達電源、驅動板 VM 或共地沒接到。");
    } else if (failed == 0) {
        Serial.println("檢查: 四輪輸出已歸零。");
    } else if (dirForced) {
        Serial.println("檢查: 電。DIR 腳被外部拉住，Pico 寫出的電位和腳上讀回的不一樣。");
    } else {
        Serial.println("檢查: 訊號。Pico 沒有把 PWM 送出去。");
    }
}

static void driveWheels(int onlyIndex, int speed) {
    for (int index = 0; index < 4; ++index)
        writeWheel(wheels[index], (onlyIndex < 0 || onlyIndex == index) ? speed : 0);
    reportWheels(onlyIndex, speed);
    reportSignal(onlyIndex, speed);
}

static void stopAllTt() {
    for (int index = 0; index < 5; ++index) {
        digitalWrite(TT_IA[index], LOW);
        digitalWrite(TT_IB[index], LOW);
    }
    ttPhase = TtPhase::Idle;
}

static void driveTt(int index, bool forward) {
    stopAllTt();
    digitalWrite(TT_IA[index], forward ? LOW : HIGH);
    digitalWrite(TT_IB[index], forward ? HIGH : LOW);
}

static void startTtGate(int index) {
    timedRun = false;
    driveTt(index, true);
    ttIndex = index;
    ttPhase = TtPhase::Opening;
    ttPhaseEndsMs = millis() + TT_MOVE_MS;
    Serial.print("TT");
    Serial.print(index + 1);
    Serial.println(" OPEN 1500 ms");
}

static void updateTtGate() {
    if (ttPhase == TtPhase::Idle || static_cast<int32_t>(millis() - ttPhaseEndsMs) < 0)
        return;
    if (ttPhase == TtPhase::Opening) {
        driveTt(ttIndex, false);
        ttPhase = TtPhase::Closing;
        ttPhaseEndsMs = millis() + TT_MOVE_MS;
        Serial.print("TT");
        Serial.print(ttIndex + 1);
        Serial.println(" CLOSE 1500 ms");
        return;
    }
    const int finished = ttIndex;
    stopAllTt();
    Serial.print("TT");
    Serial.print(finished + 1);
    Serial.println(" STOP");
}

static void stopMotor() {
    timedRun = false;
    stopAllTt();
    stopWheels();
    writeEsc(ESC_STOP_US);
    Serial.println("STOP");
}

static void drive(bool forward, uint32_t durationMs) {
    driveTt(0, forward);
    if (durationMs > 0) {
        timedRun = true;
        stopAtMs = millis() + durationMs;
    } else {
        timedRun = false;
    }
    Serial.print(forward ? "FORWARD" : "REVERSE");
    if (durationMs > 0) {
        Serial.print(" ");
        Serial.print(durationMs);
        Serial.print(" ms");
    }
    Serial.println();
}

static void writeServo(int pulseUs) {
    servoUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
    positionServo.writeMicroseconds(servoUs);
    Serial.print("SERVO ");
    Serial.print(servoUs);
    Serial.println(" us");
}

static void printHelp() {
    Serial.println("Five TT gates, 1500 ms open then 1500 ms close");
    Serial.println("  L 1 .. L 5   run that TT motor");
    Serial.println("  f / f <ms>   jog TT1 forward");
    Serial.println("  r / r <ms>   jog TT1 reverse");
    Serial.println("  s            stop TT, wheels, and ESC");
    Serial.println("DS3240 servo test, GP15, 600..2400 us");
    Serial.println("  v            center 1500 us");
    Serial.println("  v <us>       move to that pulse width");
    Serial.println("Brushless ESC test, GP14, 1100..1940 us");
    Serial.println("  b            stop 1100 us");
    Serial.println("  b <us>       throttle pulse");
    Serial.println("Four drive motors, PWM -255..255, right side mirrored");
    Serial.println("  w            all forward at 150");
    Serial.println("  w <pwm>      all wheels, negative is reverse");
    Serial.println("  1..4 <pwm>   FR FL BR BL, other wheels stop");
}

static char *skipSpace(char *text) {
    while (*text == ' ' || *text == '\t')
        ++text;
    return text;
}

static void handleLine(char *line) {
    line = skipSpace(line);
    if (*line == '\0' || strcmp(line, "h") == 0 || strcmp(line, "help") == 0) {
        printHelp();
        return;
    }
    if (strcmp(line, "s") == 0 || strcmp(line, "stop") == 0) {
        stopMotor();
        return;
    }

    if (line[0] == 'v' && (line[1] == '\0' || line[1] == ' ' || line[1] == '\t')) {
        char *argument = skipSpace(line + 1);
        if (*argument == '\0') {
            writeServo(SERVO_CENTER_US);
            return;
        }
        char *end = nullptr;
        const long value = strtol(argument, &end, 10);
        if (end == argument || *skipSpace(end) != '\0') {
            Serial.println("Servo pulse must be a number, for example v 1500.");
            return;
        }
        if (value < SERVO_MIN_US || value > SERVO_MAX_US) {
            Serial.println("Servo pulse must be from 600 to 2400 us.");
            return;
        }
        writeServo(static_cast<int>(value));
        return;
    }

    if (line[0] == 'b' && (line[1] == '\0' || line[1] == ' ' || line[1] == '\t')) {
        char *argument = skipSpace(line + 1);
        if (*argument == '\0') {
            writeEsc(ESC_STOP_US);
            return;
        }
        char *end = nullptr;
        const long value = strtol(argument, &end, 10);
        if (end == argument || *skipSpace(end) != '\0') {
            Serial.println("ESC pulse must be a number, for example b 1500.");
            return;
        }
        if (value < ESC_STOP_US || value > ESC_FULL_US) {
            Serial.println("ESC pulse must be from 1100 to 1940 us.");
            return;
        }
        writeEsc(static_cast<int>(value));
        return;
    }

    if (line[0] == 'L' || line[0] == 'l') {
        char *argument = skipSpace(line + 1);
        char *end = nullptr;
        const long motor = strtol(argument, &end, 10);
        if (end == argument || *skipSpace(end) != '\0' || motor < 1 || motor > 5) {
            Serial.println("Use L 1, L 2, L 3, L 4, or L 5.");
            return;
        }
        startTtGate(static_cast<int>(motor) - 1);
        return;
    }

    if (line[0] == 'w' || (line[0] >= '1' && line[0] <= '4')) {
        if (line[1] != '\0' && line[1] != ' ' && line[1] != '\t') {
            Serial.println("Unknown command. Type h.");
            return;
        }
        const int onlyIndex = line[0] == 'w' ? -1 : line[0] - '1';
        char *argument = skipSpace(line + 1);
        int speed = WHEEL_DEFAULT_PWM;
        if (*argument != '\0') {
            char *end = nullptr;
            const long value = strtol(argument, &end, 10);
            if (end == argument || *skipSpace(end) != '\0' ||
                value < -WHEEL_PWM_MAX || value > WHEEL_PWM_MAX) {
                Serial.println("Wheel PWM must be from -255 to 255.");
                return;
            }
            speed = static_cast<int>(value);
        }
        driveWheels(onlyIndex, speed);
        return;
    }

    const bool forward = line[0] == 'f';
    const bool reverse = line[0] == 'r';
    if (!forward && !reverse) {
        Serial.println("Unknown command. Type h.");
        return;
    }
    if (line[1] != '\0' && line[1] != ' ' && line[1] != '\t') {
        Serial.println("Unknown command. Type h.");
        return;
    }

    uint32_t durationMs = 0;
    char *argument = skipSpace(line + 1);
    if (*argument != '\0') {
        char *end = nullptr;
        const long value = strtol(argument, &end, 10);
        if (end == argument || *skipSpace(end) != '\0' || value <= 0) {
            Serial.println("Duration must be a positive number of milliseconds.");
            return;
        }
        durationMs = static_cast<uint32_t>(value);
    }
    drive(forward, durationMs);
}

static void pollSerial() {
    while (Serial.available() > 0) {
        const char character = static_cast<char>(Serial.read());
        if (character == '\r' || character == '\n') {
            if (lineLength == 0)
                continue;
            lineBuffer[lineLength] = '\0';
            handleLine(lineBuffer);
            lineLength = 0;
        } else if (lineLength < sizeof(lineBuffer) - 1U) {
            lineBuffer[lineLength++] = character;
        } else {
            lineLength = 0;
            Serial.println("Command too long.");
        }
    }
}

void setup() {
    for (int index = 0; index < 5; ++index) {
        pinMode(TT_IA[index], OUTPUT);
        pinMode(TT_IB[index], OUTPUT);
        digitalWrite(TT_IA[index], LOW);
        digitalWrite(TT_IB[index], LOW);
    }
    analogWriteFreq(WHEEL_PWM_FREQ);
    analogWriteRange(WHEEL_PWM_MAX);
    for (const Wheel &wheel : wheels) {
        pinMode(wheel.dirPin, OUTPUT);
        digitalWrite(wheel.dirPin, LOW);
    }
    stopWheels();

    pinMode(PIN_ESC, OUTPUT);
    digitalWrite(PIN_ESC, LOW);
    brushlessEsc.attach(PIN_ESC, 1000, 2000);
    brushlessEsc.writeMicroseconds(ESC_STOP_US);
    positionServo.attach(PIN_SERVO, 500, 2500);
    positionServo.writeMicroseconds(SERVO_CENTER_US);

    Serial.begin(115200);
    delay(300);
    Serial.println("tt_l9110_test: TT1-5 on GP16/17..GP27/28, GP15=servo, GP14=ESC, GP6-13=wheels.");
    Serial.println("TT stopped. Servo at 1500 us. ESC at 1100 us. Wheels stopped.");
    printHelp();
}

void loop() {
    pollSerial();
    updateTtGate();
    if (timedRun && static_cast<int32_t>(millis() - stopAtMs) >= 0)
        stopMotor();
}
