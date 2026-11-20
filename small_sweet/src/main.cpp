/* ============================================================
 *  Hobbywing SkyWalker 40A V2 ×2  +  Raspberry Pi Pico
 *  差速驅動（坦克轉向）Serial Monitor 控制
 *  ------------------------------------------------------------
 *  為什麼每顆 ESC 要接「兩條」訊號線？
 *    SkyWalker V2 是固定翼用 ESC，本身沒有車用的雙向油門。
 *    它的正反轉是靠「反推煞車 (Reverse Brake)」功能：
 *      - 白色線 = 油門 (THR)   ：1000us = 停, 2000us = 全速
 *      - 黃色線 = 方向 (REV)   ：1000us = 正轉, 2000us = 反轉
 *    前提：ESC 必須先用編程盒/遙控器把「1. 剎車類型」設成 Reverse。
 *    （出廠預設是 Disabled，沒設定的話黃線完全沒作用！）
 *
 *  接線（每顆 ESC 接到 Pico 的 4 條線）
 *    白 (油門訊號) → GP6 (左) / GP8 (右)
 *    黃 (反推訊號) → GP7 (左) / GP9 (右)
 *    黑 (GND)      → Pico GND   ← 兩顆都要接，一定要共地
 *    紅 (BEC 5V)   → 「不要接」！Pico 用 USB 或獨立 5V 供電
 *
 *  操作（Serial Monitor 115200，按鍵直接送，不需 Enter）
 *    w / f    : 前進
 *    s / b    : 後退
 *    a        : 左轉（前進中）w
 *    d        : 右轉（前進中）
 *    q        : 原地左旋
 *    e        : 原地右旋
 *    x 或空白 : 停止
 *    1 ~ 9    : 速度 (1 最慢 ~ 9 最快)
 *    0        : 速度歸零
 *    + / -    : 速度 ±5%
 *    k        : 油門行程校正 (第一次使用必做)
 *    c        : 逐顆馬達測試 (正轉 / 反轉)
 *    p        : 狀態
 *    h / ?    : 說明
 * ============================================================ */

#include <Arduino.h>
#include <Servo.h>

// ============================================================
//  1. 接線
// ============================================================
constexpr uint8_t PIN_THR_L = 6; // 左 ESC 白線（油門）
constexpr uint8_t PIN_REV_L = 7; // 左 ESC 黃線（反推）
constexpr uint8_t PIN_THR_R = 8; // 右 ESC 白線（油門）
constexpr uint8_t PIN_REV_R = 9; // 右 ESC 黃線（反推）

// ============================================================
//  2. ESC 訊號參數
// ============================================================
// 油門訊號範圍（校正後 ESC 會記住 1000~2000us）
constexpr int THR_STOP_US = 1000; // 停止
constexpr int THR_FULL_US = 2000; // 全速

// 反推方向線：0% = 正轉，100% = 反轉。
// 注意！ESC 第一次上電時這條線必須在 0%（1000us），所以開機一律送 1000。
constexpr int REV_FORWARD_US = 1000;
constexpr int REV_BACKWARD_US = 2000;

constexpr int PWM_MAX = 255;  // 內部運算用的滿刻度
constexpr int MIN_MAG = 12;   // 低於這個量直接視為 0，避免無刷馬達只發熱不轉

// 方向切換保護：ESC 反轉時會「先停再反轉」，我們也照做以免大電流。
constexpr uint32_t DIR_STOP_MS = 250;    // 油門歸零後等馬達真的停
constexpr uint32_t DIR_SETTLE_MS = 400;  // 切完方向後的空窗，才開始給油門

// 加減速斜率
constexpr int RAMP_STEP = 8;
constexpr uint32_t RAMP_INTERVAL_MS = 6;

// 安全機制：超過這個時間沒收到指令就自動停止。0 = 關閉。
constexpr uint32_t COMMAND_TIMEOUT_MS = 0;

// 開機時的主動脈衝時間（ms），讓 Pico 先穩定輸出「停止」訊號再讓使用者上電。
constexpr uint32_t BOOT_ARM_DELAY_MS = 1000;

// ============================================================
//  3. ESC 物件
// ============================================================
struct Esc {
    uint8_t thrPin;
    uint8_t revPin;
    Servo thr;
    Servo rev;

    int wantDir = 1;      // 期望方向：+1 前進, -1 反轉
    int wantMag = 0;      // 期望油門量 0~255
    int dir = 1;          // ESC 目前收到的方向
    int mag = 0;          // ESC 目前收到的油門量
    uint32_t t = 0;       // 狀態機計時
    uint32_t settleUntil = 0;
};

Esc escL{PIN_THR_L, PIN_REV_L};
Esc escR{PIN_THR_R, PIN_REV_R};
Esc* const escs[] = {&escL, &escR};

// ============================================================
//  4. 狀態
// ============================================================
enum class Mode { Stop, Forward, Backward, TurnLeft, TurnRight, SpinLeft, SpinRight };

Mode driveMode = Mode::Stop;
int speedSetting = 150; // 0 ~ 255
uint32_t lastCommandMs = 0;
uint32_t lastRampMs = 0;

// ============================================================
//  5. 底層輸出
// ============================================================
void writeThrottle(Esc& e, int mag) {
    mag = constrain(mag, 0, PWM_MAX);
    if(mag < MIN_MAG)
        mag = 0;
    e.thr.writeMicroseconds(THR_STOP_US + mag * (THR_FULL_US - THR_STOP_US) / PWM_MAX);
}

void writeDir(Esc& e, int dir) {
    e.rev.writeMicroseconds((dir < 0) ? REV_BACKWARD_US : REV_FORWARD_US);
}

void stopAllOutputs() {
    for(Esc* e : escs) {
        e->dir = 1;
        e->mag = 0;
        e->wantDir = 1;
        e->wantMag = 0;
        e->settleUntil = 0;
        writeDir(*e, 1);
        writeThrottle(*e, 0);
    }
}

// 每顆 ESC 一個小狀態機：轉向不同 -> 先歸零 -> 等停穩 -> 切方向 -> 空窗 -> 給油門
void updateEsc(Esc& e, uint32_t now) {
    // (1) 要換方向
    if(e.wantDir != e.dir) {
        if(e.mag > 0) {
            e.mag = 0;
            writeThrottle(e, 0);
            e.t = now;
            return;
        }
        if(now - e.t >= DIR_STOP_MS) {
            e.dir = e.wantDir;
            writeDir(e, e.dir);
            e.settleUntil = now + DIR_SETTLE_MS;
            e.t = now;
        }
        return;
    }

    // (2) 剛切完方向，先不要給油門
    if(now < e.settleUntil)
        return;

    // (3) 依斜率加減速
    int target = e.wantMag;
    int next = e.mag;
    if(next < target)
        next = min(next + RAMP_STEP, target);
    else if(next > target)
        next = max(next - RAMP_STEP, target);

    if(next != e.mag) {
        e.mag = next;
        writeThrottle(e, e.mag);
    }
}

// ============================================================
//  6. 差速混控
// ============================================================
const char* modeName() {
    switch(driveMode) {
    case Mode::Forward: return "前進 (FORWARD)";
    case Mode::Backward: return "後退 (BACKWARD)";
    case Mode::TurnLeft: return "左轉 (TURN L)";
    case Mode::TurnRight: return "右轉 (TURN R)";
    case Mode::SpinLeft: return "原地左旋 (SPIN L)";
    case Mode::SpinRight: return "原地右旋 (SPIN R)";
    default: return "停止 (STOP)";
    }
}

// 回傳 v(前進量) 與 t(右轉量)，範圍 -1.0 ~ 1.0
void mixFor(Mode m, float& v, float& t) {
    switch(m) {
    case Mode::Forward: v = 1.00f; t = 0.00f; break;
    case Mode::Backward: v = -1.00f; t = 0.00f; break;
    // 前進中轉彎：外輪全速、內輪 30%
    case Mode::TurnLeft: v = 0.65f; t = -0.35f; break;
    case Mode::TurnRight: v = 0.65f; t = 0.35f; break;
    // 原地旋轉
    case Mode::SpinLeft: v = 0.0f; t = -1.0f; break;
    case Mode::SpinRight: v = 0.0f; t = 1.0f; break;
    default: v = 0.0f; t = 0.0f; break;
    }
}

void applyDrive() {
    float v, t;
    mixFor(driveMode, v, t);

    float l = constrain(v + t, -1.0f, 1.0f); // 左輪 = v + t
    float r = constrain(v - t, -1.0f, 1.0f); // 右輪 = v - t

    int li = (int)(l * speedSetting);
    int ri = (int)(r * speedSetting);

    escL.wantDir = (li < 0) ? -1 : 1;
    escL.wantMag = abs(li);
    escR.wantDir = (ri < 0) ? -1 : 1;
    escR.wantMag = abs(ri);

    lastCommandMs = millis();
}

void stopAll() {
    driveMode = Mode::Stop;
    for(Esc* e : escs) {
        e->wantMag = 0;
        e->wantDir = 1;
    }
    lastCommandMs = millis();
}

// ============================================================
//  7. 訊息
// ============================================================
void printStatus() {
    Serial.print("[狀態] ");
    Serial.print(modeName());
    Serial.print("  速度=");
    Serial.print(speedSetting);
    Serial.print(" (");
    Serial.print(speedSetting * 100 / PWM_MAX);
    Serial.print("%)  左=");
    Serial.print(escL.dir < 0 ? "-" : "+");
    Serial.print(escL.mag);
    Serial.print(" 右=");
    Serial.print(escR.dir < 0 ? "-" : "+");
    Serial.println(escR.mag);
}

void printHelp() {
    Serial.println();
    Serial.println("===== SkyWalker 40A V2 ×2 控制 =====");
    Serial.println("  w / f    : 前進");
    Serial.println("  s / b    : 後退");
    Serial.println("  a        : 左轉");
    Serial.println("  d        : 右轉");
    Serial.println("  q        : 原地左旋");
    Serial.println("  e        : 原地右旋");
    Serial.println("  x / 空白 : 停止");
    Serial.println("  1 ~ 9    : 速度 (1 最慢 ~ 9 最快)");
    Serial.println("  0        : 速度歸零");
    Serial.println("  + / -    : 速度 +5% / -5%");
    Serial.println("  k        : 油門行程校正 (第一次使用必做)");
    Serial.println("  c        : 逐顆馬達測試");
    Serial.println("  p        : 顯示狀態");
    Serial.println("  h / ?    : 顯示說明");
    Serial.println("===================================");
}

// ============================================================
//  8. 校正與測試
// ============================================================
// 油門行程校正：ESC 出廠預設是 1100~1940us (Futaba)，
// 必須讓它重新學會 Pico 的 1000~2000us。
void calibrateThrottle() {
    Serial.println();
    Serial.println("========== 油門行程校正 ==========");
    Serial.println(" 1. 先確認 ESC 的電池『還沒接上』");
    Serial.println(" 2. 程式現在會持續送出『最大油門 2000us』");
    Serial.println(" 3. 請接上 ESC 電池，會聽到『嗶-嗶-』兩聲");
    Serial.println(" 4. 聽到後『立刻按 Enter』");
    Serial.println(" 5. 程式送出『最低油門 1000us』，");
    Serial.println("    聽到『嗶 N 聲 + 一長聲』就完成了");
    Serial.println("==================================");

    // 送最大油門，等使用者按 Enter
    for(Esc* e : escs)
        e->thr.writeMicroseconds(THR_FULL_US);
    Serial.println(">>> 目前輸出 2000us，請接電池並在聽到兩聲短嗶後按 Enter...");

    uint32_t t0 = millis();
    bool pressed = false;
    while(millis() - t0 < 15000) {
        if(Serial.available()) {
            while(Serial.available())
                Serial.read();
            pressed = true;
            break;
        }
        delay(20);
    }
    if(!pressed)
        Serial.println(">>> 逾時（15 秒），直接進入下一步。可以按 k 重來。");

    // 送最低油門
    for(Esc* e : escs)
        e->thr.writeMicroseconds(THR_STOP_US);
    Serial.println(">>> 目前已輸出 1000us，請聽『嗶 N 聲 + 長嗶』...");
    delay(2500);

    // 重設內部狀態
    stopAllOutputs();
    lastRampMs = millis();
    Serial.println(">>> 校正流程結束，馬達應該停住。");
    printStatus();
}

// 逐顆測試：單顆慢速正轉、反轉，用來確認接線與方向。
// 請先把車子架空。若某顆的「正轉」方向相反，把該馬達三條線中的任兩條對調即可。
void calibrateMotors() {
    Serial.println();
    Serial.println("[測試] 依序單顆慢速正轉 1.5 秒 -> 停 -> 反轉 1.5 秒");
    Serial.println("       重點：兩顆的『正轉』必須是同一個物理方向。");
    const char* names[] = {"左 (GP6/GP7)", "右 (GP8/GP9)"};

    for(int i = 0; i < 2; i++) {
        Esc& e = *escs[i];
        for(int pass = 0; pass < 2; pass++) {
            int d = (pass == 0) ? 1 : -1;
            Serial.print(pass == 0 ? "  正轉 -> " : "  反轉 -> ");
            Serial.println(names[i]);

            e.wantDir = d;
            e.wantMag = 0;
            // 等方向切換完成
            uint32_t w = millis();
            while(millis() - w < 1200) {
                updateEsc(e, millis());
                delay(5);
            }
            e.wantMag = 120;
            w = millis();
            while(millis() - w < 1500) {
                updateEsc(e, millis());
                delay(5);
            }
            e.wantMag = 0;
            w = millis();
            while(millis() - w < 600) {
                updateEsc(e, millis());
                delay(5);
            }
        }
    }
    stopAllOutputs();
    Serial.println("[測試] 結束，馬達已停止");
    printStatus();
}

// ============================================================
//  9. 指令
// ============================================================
void setSpeed(int value) {
    int old = speedSetting;
    speedSetting = constrain(value, 0, PWM_MAX);
    if(speedSetting != old && driveMode != Mode::Stop)
        applyDrive();
    printStatus();
}

void handleCommand(char c) {
    switch(c) {
    case 'w': case 'W': case 'f': case 'F':
        driveMode = Mode::Forward;
        applyDrive();
        printStatus();
        break;

    case 's': case 'S': case 'b': case 'B':
        driveMode = Mode::Backward;
        applyDrive();
        printStatus();
        break;

    case 'a': case 'A':
        driveMode = Mode::TurnLeft;
        applyDrive();
        printStatus();
        break;

    case 'd': case 'D':
        driveMode = Mode::TurnRight;
        applyDrive();
        printStatus();
        break;

    case 'q': case 'Q':
        driveMode = Mode::SpinLeft;
        applyDrive();
        printStatus();
        break;

    case 'e': case 'E':
        driveMode = Mode::SpinRight;
        applyDrive();
        printStatus();
        break;

    case 'x': case 'X': case ' ':
        stopAll();
        printStatus();
        break;

    case 'k': case 'K':
        stopAll();
        delay(600);
        calibrateThrottle();
        break;

    case 'c': case 'C':
        stopAll();
        delay(600);
        calibrateMotors();
        break;

    case 'p': case 'P':
        printStatus();
        break;

    case 'h': case 'H': case '?':
        printHelp();
        break;

    case '+': case '=':
        setSpeed(speedSetting + PWM_MAX / 20);
        break;

    case '-': case '_':
        setSpeed(speedSetting - PWM_MAX / 20);
        break;

    default:
        if(c >= '0' && c <= '9') {
            if(c == '0')
                setSpeed(0);
            else
                setSpeed((c - '0') * PWM_MAX / 9); // 1 -> 28 ... 9 -> 255
        }
        break;
    }
}

// ============================================================
//  10. Arduino
// ============================================================
void setup() {
    // 先把 4 條訊號線接上並輸出「停止 / 正轉」，
    // 這樣如果 ESC 和 Pico 同時上電，ESC 也能正常 arm。
    escL.thr.attach(PIN_THR_L);
    escL.rev.attach(PIN_REV_L);
    escR.thr.attach(PIN_THR_R);
    escR.rev.attach(PIN_REV_R);
    stopAllOutputs();

    // 讓 ESC 有時間看到穩定的低油門訊號再 arm
    delay(BOOT_ARM_DELAY_MS);

    Serial.begin(115200);
    delay(300);
    printHelp();
    Serial.println("提示：第一次使用請先按 k 做油門行程校正。");
    printStatus();
}

void loop() {
    while(Serial.available() > 0) {
        char c = (char)Serial.read();
        if(c == '\n' || c == '\r')
            continue;
        handleCommand(c);
    }

    uint32_t now = millis();
    if(now - lastRampMs >= RAMP_INTERVAL_MS) {
        lastRampMs = now;
        for(Esc* e : escs)
            updateEsc(*e, now);
    }

    if(COMMAND_TIMEOUT_MS > 0 && driveMode != Mode::Stop &&
       now - lastCommandMs > COMMAND_TIMEOUT_MS) {
        stopAll();
        Serial.println("[安全] 太久沒收到指令，自動停止");
        printStatus();
    }
}
