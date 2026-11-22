/**
 * @file    doppler_platoon.ino
 * @brief   Doppler Radar Collision Avoidance — Trailing Vehicle Firmware
 * @version 2.0
 *
 * Hardware:
 *   HB100  X-band Doppler module    →  U1 (LM358P) amp/env
 *   U3/U4  HLF393N dual comparators →  SQ signal + TRIG
 *   U5     SN74HCT74N D flip-flop   →  hardware emergency latch
 *   U9     NE555 (monostable)       →  dead-time ~0.8 s
 *   U10    NE555 (monostable)       →  reverse pulse ~0.4 s
 *   U6     SN74HC08N AND gate       →  drive gating (FWD / BRK / REVP)
 *   TB6612FNG motor driver          →  traction motor
 *   ESP32 (38-pin dev board)
 *
 * Power rails:
 *   BAT_SW (11.1 V LiPo) → LM2596 → 5 V → 1N4007 → 5VA (~4.69 V)
 *   VG ≈ 2.35 V  (10 kΩ / 10 kΩ divider from 5VA)
 *   3V3L = 3.3 V from ESP32 on-board regulator
 *   Q_3V3 = 5 V × 2 / (1+2) ≈ 3.33 V  (1 kΩ / 2 kΩ divider from U5 1Q)
 *
 * GPIO assignment:
 *   GPIO32  IN   SQ         Doppler square wave (ISR RISING, freq measure)
 *   GPIO35  IN   TRIG       Threshold trigger from U3 Pin 7  [INPUT_ONLY]
 *   GPIO34  IN   Q_3V3      U5 1Q via 1kΩ/2kΩ divider       [INPUT_ONLY]
 *   GPIO26  OUT  CMD_FWD    Forward drive command → U6 1A
 *   GPIO33  OUT  V2V_TRIG   SW / V2V trigger → 1N4148 → CLK_NODE
 *   GPIO25  OUT  PWM        Speed PWM → TB6612 PWMA/PWMB
 *
 * Serial protocol (115200 baud):
 *   TX telemetry @ TELEM_HZ (default 10 Hz):
 *     freq=<Hz> spd=<m/s> gap_est=<m> latch=<0|1> trig=<0|1> fwd=<0|1> drv=<state>
 *
 *   RX commands (single char):
 *     T  — pulse GPIO33 (software / V2V trigger)
 *     F  — set CMD_FWD HIGH  (forward allow)
 *     B  — set CMD_FWD LOW   (forward block)
 *     R  — print latch clear reminder (latch is hardware-only via 1/CLR pin)
 *     S  — print current state summary
 *     ?  — print this help
 *
 * Notes:
 *   - GPIO34 & GPIO35 are input-only on ESP32; NO pull-up/down available.
 *     Use external 47 kΩ pull-down on CLK_NODE as designed.
 *   - Latch clear is HARDWARE ONLY: momentary pull-down on U5 Pin 1 (1/CLR).
 *     Do NOT wire ESP32 EN to 1/CLR — boot noise will false-trigger the latch.
 *   - The 4.7 kΩ + 1 µF RC on 1/CLR provides power-on reset only.
 */

// ═══════════════════════════════════════════════════════
//  CONFIGURATION — adjust to match your hardware
// ═══════════════════════════════════════════════════════

// ── Pin definitions ──────────────────────────────────
#define PIN_SQ          32
#define PIN_TRIG        35
#define PIN_Q3V3        34
#define PIN_CMD_FWD     26
#define PIN_SW_TRIG     33
#define PIN_PWM         25

// ── HB100 Doppler constant ───────────────────────────
// f_doppler = 2 * f_carrier * v / c
// f_carrier = 10.525 GHz  →  k = 2 * 10.525e9 / 3e8 = 70.167 ≈ 70.42 Hz/(m/s)
// Measure your module with a known speed if you need better accuracy.
#define HB100_K         70.42f      // Hz per m/s

// ── Drive thresholds ─────────────────────────────────
#define BRAKE_SPD_MPS   0.50f       // m/s closing speed → begin PWM reduction
#define MAX_CLOSING_MPS 3.00f       // m/s clamp for PWM calculation

// ── PWM ──────────────────────────────────────────────
#define PWM_CHANNEL     0
#define PWM_FREQ_HZ     20000       // 20 kHz (inaudible)
#define PWM_RESOLUTION  8           // bits → 0-255
#define PWM_NORMAL      200         // duty at zero closing speed
#define PWM_MIN         20          // minimum duty when braking

// ── Telemetry ────────────────────────────────────────
#define TELEM_HZ        10          // serial lines per second
#define BAUD_RATE       115200

// ── Timing ───────────────────────────────────────────
#define SQ_TIMEOUT_US   500000UL    // 0.5 s without pulse → speed = 0
#define SW_TRIG_US      500         // GPIO33 pulse width (µs)

// ── Moving average filter depth ──────────────────────
#define FREQ_AVG_N      8           // samples for Doppler smoothing


// ═══════════════════════════════════════════════════════
//  DRIVE STATE MACHINE
// ═══════════════════════════════════════════════════════
typedef enum {
    STATE_NORMAL,       // FWD allowed, PWM proportional
    STATE_LATCH_SET,    // latch fired, waiting for hardware timers
    STATE_BRAKE,        // U10 pulse expired, hold BRK
} DriveState;

static const char* driveStateStr[] = { "NORMAL", "LATCH_SET", "BRAKE" };


// ═══════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════

// ISR data
volatile uint32_t g_sqRiseUs    = 0;
volatile uint32_t g_sqPeriodUs  = 0;
volatile bool     g_sqFresh     = false;

// Moving average ring buffer
uint32_t g_freqBuf[FREQ_AVG_N]  = {0};
uint8_t  g_freqIdx              = 0;
uint8_t  g_freqCount            = 0;

// Processed values
float    g_dopplerHz    = 0.0f;
float    g_relSpeedMps  = 0.0f;
float    g_gapEstM      = 2.0f;     // estimated gap (simple integrator)

// State
DriveState g_drive      = STATE_NORMAL;
bool       g_latchPrev  = false;

// Timers
uint32_t   g_telemMs    = 0;
uint32_t   g_loopMs     = 0;


// ═══════════════════════════════════════════════════════
//  ISR — rising edge on SQ (GPIO32)
// ═══════════════════════════════════════════════════════
void IRAM_ATTR onSqRise()
{
    uint32_t now = micros();
    if (g_sqRiseUs > 0) {
        g_sqPeriodUs = now - g_sqRiseUs;
        g_sqFresh    = true;
    }
    g_sqRiseUs = now;
}


// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup()
{
    Serial.begin(BAUD_RATE);
    delay(100);

    Serial.println(F("\n╔══════════════════════════════════════╗"));
    Serial.println(F("║  Doppler Platoon Firmware  v2.0      ║"));
    Serial.println(F("║  HB100 + ESP32 + 74HCT74 + NE555     ║"));
    Serial.println(F("╚══════════════════════════════════════╝"));

    // ── GPIO config ──
    pinMode(PIN_SQ,      INPUT);
    pinMode(PIN_TRIG,    INPUT);        // INPUT_ONLY on ESP32 — no pullup
    pinMode(PIN_Q3V3,    INPUT);        // INPUT_ONLY on ESP32 — no pullup
    pinMode(PIN_CMD_FWD, OUTPUT);
    pinMode(PIN_SW_TRIG, OUTPUT);

    // Safe power-on defaults
    digitalWrite(PIN_CMD_FWD, HIGH);    // allow forward drive
    digitalWrite(PIN_SW_TRIG, LOW);     // no trigger

    // ── PWM ──
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, PWM_NORMAL);

    // ── ISR ──
    attachInterrupt(digitalPinToInterrupt(PIN_SQ), onSqRise, RISING);

    Serial.printf("Pins   : SQ=%d TRIG=%d Q=%d FWD=%d SWTRIG=%d PWM=%d\n",
                  PIN_SQ, PIN_TRIG, PIN_Q3V3, PIN_CMD_FWD, PIN_SW_TRIG, PIN_PWM);
    Serial.printf("HB100 k: %.2f Hz/(m/s)\n", HB100_K);
    Serial.printf("Telem  : %d Hz @ %d baud\n", TELEM_HZ, BAUD_RATE);
    Serial.println(F("Commands: T=SW-trigger  F=FWD-hi  B=FWD-lo  S=status  ?=help"));
    Serial.println(F("--- LOOP START ---\n"));
}


// ═══════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════
void loop()
{
    uint32_t now = millis();
    float    dt  = (now - g_loopMs) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;  // guard
    g_loopMs = now;

    // ─────────────────────────────────────────────────
    // 1. Doppler frequency measurement (ISR → main)
    // ─────────────────────────────────────────────────
    if (g_sqFresh)
    {
        noInterrupts();
        uint32_t p = g_sqPeriodUs;
        g_sqFresh  = false;
        interrupts();

        if (p > 100 && p < 5000000UL)   // 0.2 Hz – 10 kHz sanity window
        {
            // Add to moving average
            g_freqBuf[g_freqIdx % FREQ_AVG_N] = p;
            g_freqIdx++;
            if (g_freqCount < FREQ_AVG_N) g_freqCount++;

            // Compute average period → average frequency
            uint32_t sum = 0;
            for (uint8_t i = 0; i < g_freqCount; i++)
                sum += g_freqBuf[i];
            float avgPeriod = (float)sum / g_freqCount;

            g_dopplerHz   = 1e6f / avgPeriod;
            g_relSpeedMps = g_dopplerHz / HB100_K;
        }
    }

    // Timeout: no pulse for SQ_TIMEOUT_US → speed = 0
    if ((uint32_t)(micros() - g_sqRiseUs) > SQ_TIMEOUT_US)
    {
        g_dopplerHz   = 0.0f;
        g_relSpeedMps = 0.0f;
        g_freqCount   = 0;              // flush average on timeout
        g_freqIdx     = 0;
    }

    // ─────────────────────────────────────────────────
    // 2. Read hardware signal lines
    // ─────────────────────────────────────────────────
    bool trigNow  = digitalRead(PIN_TRIG);
    bool latchNow = digitalRead(PIN_Q3V3);

    // ─────────────────────────────────────────────────
    // 3. Latch event detection (rising edge on Q_3V3)
    // ─────────────────────────────────────────────────
    if (latchNow && !g_latchPrev)
    {
        g_drive = STATE_LATCH_SET;
        digitalWrite(PIN_CMD_FWD, LOW);     // redundant – U6 AND gate already blocks
        Serial.printf("[LATCH SET] speed=%.3f m/s  freq=%.1f Hz\n",
                      g_relSpeedMps, g_dopplerHz);
    }
    // Latch cleared by hardware (1/CLR pulled low) — detect falling edge
    if (!latchNow && g_latchPrev)
    {
        g_drive = STATE_NORMAL;
        digitalWrite(PIN_CMD_FWD, HIGH);
        Serial.println(F("[LATCH CLEAR] CMD_FWD restored"));
    }
    g_latchPrev = latchNow;

    // ─────────────────────────────────────────────────
    // 4. Drive output (normal operation only)
    // ─────────────────────────────────────────────────
    if (!latchNow)
    {
        // Proportional PWM braking based on closing speed
        int duty = PWM_NORMAL;
        if (g_relSpeedMps > BRAKE_SPD_MPS)
        {
            float factor = 1.0f - (g_relSpeedMps - BRAKE_SPD_MPS)
                                  / (MAX_CLOSING_MPS - BRAKE_SPD_MPS);
            duty = constrain((int)(PWM_NORMAL * factor), PWM_MIN, PWM_NORMAL);
        }
        ledcWrite(PWM_CHANNEL, duty);
        digitalWrite(PIN_CMD_FWD, HIGH);
    }

    // ─────────────────────────────────────────────────
    // 5. Gap estimator (simple dead-reckoning)
    //    Real system would use a separate distance sensor.
    // ─────────────────────────────────────────────────
    if (!latchNow)
        g_gapEstM -= g_relSpeedMps * dt;
    else
        g_gapEstM += 0.5f * dt;         // assume hardware REV pulse opens gap
    g_gapEstM = constrain(g_gapEstM, 0.05f, 10.0f);

    // ─────────────────────────────────────────────────
    // 6. Serial telemetry
    // ─────────────────────────────────────────────────
    if (now - g_telemMs >= (1000 / TELEM_HZ))
    {
        g_telemMs = now;
        Serial.printf(
            "freq=%4d Hz  spd=%5.3f m/s  gap_est=%5.2f m"
            "  latch=%d  trig=%d  fwd=%d  drv=%s\n",
            (int)g_dopplerHz,
            g_relSpeedMps,
            g_gapEstM,
            (int)latchNow,
            (int)trigNow,
            (int)digitalRead(PIN_CMD_FWD),
            driveStateStr[g_drive]
        );
    }

    // ─────────────────────────────────────────────────
    // 7. Serial command handler
    // ─────────────────────────────────────────────────
    while (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
            case 'T':
                swTrigger();
                break;

            case 'F':
                if (!latchNow) {
                    digitalWrite(PIN_CMD_FWD, HIGH);
                    Serial.println(F("[CMD] CMD_FWD → HIGH"));
                } else {
                    Serial.println(F("[CMD] Ignored — latch is set"));
                }
                break;

            case 'B':
                digitalWrite(PIN_CMD_FWD, LOW);
                Serial.println(F("[CMD] CMD_FWD → LOW"));
                break;

            case 'R':
                Serial.println(F("[INFO] Latch clear is HARDWARE-ONLY."));
                Serial.println(F("       Pull U5 Pin 1 (1/CLR) LOW momentarily."));
                break;

            case 'S':
                printStatus(latchNow, trigNow);
                break;

            case '?':
                printHelp();
                break;

            default:
                break;
        }
    }
}


// ═══════════════════════════════════════════════════════
//  HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

/**
 * @brief  Pulse GPIO33 HIGH for SW_TRIG_US to generate a CLK edge on U5.
 *         Diode-OR'd with U3 TRIG output at CLK_NODE (47 kΩ pull-down).
 */
void swTrigger()
{
    Serial.println(F("[SW TRIG] GPIO33 pulse → CLK_NODE → U5 CLK"));
    digitalWrite(PIN_SW_TRIG, HIGH);
    delayMicroseconds(SW_TRIG_US);
    digitalWrite(PIN_SW_TRIG, LOW);
}

void printStatus(bool latchNow, bool trigNow)
{
    Serial.println(F("\n--- STATUS ---"));
    Serial.printf("  Doppler freq  : %.1f Hz\n", g_dopplerHz);
    Serial.printf("  Relative speed: %.3f m/s\n", g_relSpeedMps);
    Serial.printf("  Gap estimate  : %.2f m\n", g_gapEstM);
    Serial.printf("  TRIG (GPIO35) : %s\n", trigNow  ? "HIGH" : "LOW");
    Serial.printf("  1Q   (GPIO34) : %s\n", latchNow ? "HIGH (LATCHED)" : "LOW");
    Serial.printf("  CMD_FWD       : %s\n", digitalRead(PIN_CMD_FWD) ? "HIGH" : "LOW");
    Serial.printf("  Drive state   : %s\n", driveStateStr[g_drive]);
    Serial.println(F("--------------\n"));
}

void printHelp()
{
    Serial.println(F("\n--- SERIAL COMMANDS ---"));
    Serial.println(F("  T  Pulse GPIO33 (SW / V2V trigger)"));
    Serial.println(F("  F  Set CMD_FWD HIGH  (forward allow)"));
    Serial.println(F("  B  Set CMD_FWD LOW   (forward block)"));
    Serial.println(F("  R  Remind: latch clear is hardware-only"));
    Serial.println(F("  S  Print full status"));
    Serial.println(F("  ?  Print this help"));
    Serial.println(F("-----------------------\n"));
}
