/**
 * =============================================================
 *  PS4 DualShock4  →  TMC2209 (UART)  →  NEMA 17   +   ADXL345 (IMU)
 *  ESP32 | Bluepad32 framework
 * =============================================================
 *  Basado en Test_PS4/ps4.ino (versión "evitar desconexiones").
 *  Añade:
 *    · Configuración del TMC2209 por UART (corriente, microstepping,
 *      StealthChop por software — ver nota sobre el Vref en tmcBegin()).
 *    · Lectura del acelerómetro ADXL345 por I²C (pitch/roll).
 *
 *  ── Pines ESP32 ─────────────────────────────────────────────
 *    STEP    → GPIO 5      DIR     → GPIO 18     ENABLE  → GPIO 19 (activo LOW)
 *    UART_TX → GPIO 17  ─┐ (vía R5 1k)                  ┌─ PDN_UART del TMC2209
 *    UART_RX → GPIO 16  ─┴─────── nodo UART_PDN ────────┘ (R4 10k pull-up a 3V3)
 *    I2C SDA → GPIO 21     I2C SCL → GPIO 22     ADXL INT1 → GPIO 4 (opcional)
 *
 *  Triggers PS4:
 *    R2 → adelante     L2 → atrás     (ambos → gana el mayor)
 *
 *  Librerías necesarias (Library Manager de Arduino):
 *    · Bluepad32   (board "ESP32" del core Bluepad32)
 *    · TMCStepper  (teemuatlut)
 *    · El ADXL345 se lee con Wire directo (sin librería extra).
 * =============================================================
 */

#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <math.h>

// ── Modo de control del driver (ELIGE AQUÍ) ──────────────────
//   USE_UART 0 → STEP/DIR/EN puro: NO necesita R4/R5. Corriente por el
//                potenciómetro Vref del módulo; microstepping por MS1/MS2.
//   USE_UART 1 → control avanzado por UART: necesita R4 (10k) + R5 (1k).
#define USE_UART              0
#define IMU_INTERNAL_PULLUPS  1   // 1 = pull-ups internos del ESP32 para I2C (sin R1/R2)

#if USE_UART
#include <TMCStepper.h>
#endif

// ── Pines stepper ────────────────────────────────────────────
#define PIN_STEP     4
#define PIN_DIR      2
#define PIN_ENABLE   16

// ── Pines UART del TMC2209 (Serial2) ─────────────────────────
#define PIN_UART_TX  17    // → R5(1k) → PDN_UART
#define PIN_UART_RX  16    // ← nodo UART_PDN (R4 10k pull-up a 3V3)

// ── Pines I²C de la IMU ──────────────────────────────────────
#define PIN_SDA      21
#define PIN_SCL      22
#define PIN_IMU_INT1 4     // no usado (lectura por polling)

// ── Configuración del TMC2209 (solo si USE_UART) ─────────────
#if USE_UART
#define TMC_SERIAL        Serial2
#define R_SENSE           0.11f    // ⚠ VERIFICA el sense resistor de TU módulo (0.11 típico BTT/FYSETC)
#define DRIVER_ADDRESS    0b00     // MS1=GND, MS2=GND → dirección UART 0
#define MOTOR_CURRENT_MA  600      // corriente RMS por fase — ajústala a tu NEMA17
#define MICROSTEPS        8        // debe coincidir con la lógica de velocidad de abajo
#define USE_STEALTHCHOP   true     // true = silencioso (StealthChop), false = SpreadCycle (más par a alta vel.)

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);
bool tmcOk = false;
#endif

// ── Configuración de la IMU (ADXL345) ────────────────────────
#define ADXL345_ADDR      0x53     // SDO a GND → 0x53 (a 3V3 sería 0x1D)
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_DATAX0      0x32
bool imuOk = false;

// Seguridad opcional por inclinación (robot trepador):
//   1 = corta el motor si la inclinación supera MAX_TILT_DEG. 0 = solo monitorea.
#define USE_TILT_SAFETY   0
#define MAX_TILT_DEG      70.0f

// ── Parámetros del stepper (idénticos al ps4.ino nuevo) ──────
#define MIN_STEP_US      400
#define MAX_STEP_US     1800
#define TRIGGER_DEADZONE  40
#define ACCEL_STEP_US    10

// ── Variables globales ───────────────────────────────────────
GamepadPtr connectedGamepad = nullptr;

volatile bool  motorEnabled   = false;
volatile bool  motorDir       = true;
volatile bool  tiltLockout     = false;   // la pone la seguridad por inclinación
volatile long  targetPeriodUs  = MAX_STEP_US;
volatile long  currentPeriodUs = MAX_STEP_US;

TaskHandle_t stepperTaskHandle = nullptr;

// ── ADXL345: helpers de I²C ──────────────────────────────────
void adxlWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool adxlReadXYZ(int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL_REG_DATAX0);
    if (Wire.endTransmission(false) != 0) return false;       // repeated start
    if (Wire.requestFrom((int)ADXL345_ADDR, 6) != 6) return false;
    x = (int16_t)(Wire.read() | (Wire.read() << 8));
    y = (int16_t)(Wire.read() | (Wire.read() << 8));
    z = (int16_t)(Wire.read() | (Wire.read() << 8));
    return true;
}

bool adxlBegin() {
    Wire.beginTransmission(ADXL345_ADDR);
    if (Wire.endTransmission() != 0) return false;            // ¿responde en 0x53?
    adxlWrite(ADXL_REG_DATA_FORMAT, 0x0B);                    // full-res, ±16 g
    adxlWrite(ADXL_REG_POWER_CTL,   0x08);                    // measure mode
    return true;
}

// ── Callbacks Bluepad32 ──────────────────────────────────────
void onConnectedGamepad(GamepadPtr gp) {
    connectedGamepad = gp;
    Serial.println("✅ PS4 conectado");
    gp->setRumble(0x80, 0x40);
    delay(200);
    gp->setRumble(0, 0);
}

void onDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("❌ PS4 desconectado");
    connectedGamepad = nullptr;
    motorEnabled = false;
    digitalWrite(PIN_ENABLE, HIGH);
}

// ── Mapeo de trigger a período (curva sqrt, escala 0-1023) ───
long triggerToPeriod(int triggerVal) {
    if (triggerVal < TRIGGER_DEADZONE) return MAX_STEP_US;
    float t = (float)(triggerVal - TRIGGER_DEADZONE) / (1023.0f - TRIGGER_DEADZONE);
    t = constrain(t, 0.0f, 1.0f);
    t = sqrtf(t);
    long period = (long)(MAX_STEP_US - t * (MAX_STEP_US - MIN_STEP_US));
    return constrain(period, MIN_STEP_US, MAX_STEP_US);
}

// ── Tarea del stepper (Core 0) ───────────────────────────────
void stepperTask(void* param) {
    digitalWrite(PIN_ENABLE, HIGH);
    digitalWrite(PIN_STEP, LOW);

    int stepCounter = 0;

    while (true) {
        if (!motorEnabled || tiltLockout) {
            digitalWrite(PIN_ENABLE, HIGH);
            digitalWrite(PIN_STEP, LOW);
            currentPeriodUs = MAX_STEP_US;
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        digitalWrite(PIN_ENABLE, LOW);
        digitalWrite(PIN_DIR, motorDir ? HIGH : LOW);
        delayMicroseconds(5);

        if (currentPeriodUs > targetPeriodUs) {
            currentPeriodUs -= ACCEL_STEP_US;
            if (currentPeriodUs < targetPeriodUs) currentPeriodUs = targetPeriodUs;
        } else if (currentPeriodUs < targetPeriodUs) {
            currentPeriodUs += ACCEL_STEP_US;
            if (currentPeriodUs > targetPeriodUs) currentPeriodUs = targetPeriodUs;
        }

        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(2);
        digitalWrite(PIN_STEP, LOW);

        long waitUs = currentPeriodUs - 2;
        if (waitUs > 0) delayMicroseconds(waitUs);

        stepCounter++;
        if (stepCounter >= 50) {   // cede CPU al stack BT para no desconectarse
            stepCounter = 0;
            vTaskDelay(1);
        }
    }
}

// ── Inicialización del TMC2209 por UART ──────────────────────
#if USE_UART
void tmcBegin() {
    TMC_SERIAL.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    driver.begin();
    driver.toff(5);                       // habilita el chopper (necesario para mover)
    // NOTA Vref: por defecto el pin VREF (potenciómetro) ESCALA la corriente.
    // Para que rms_current() mande de verdad, sube el pot a ~tope, o descomenta
    // la línea de abajo para usar la referencia interna (si tu versión de TMCStepper la trae):
    // driver.I_scale_analog(false);
    driver.rms_current(MOTOR_CURRENT_MA); // corriente RMS por fase por software
    driver.microsteps(MICROSTEPS);
    driver.en_spreadCycle(!USE_STEALTHCHOP);
    driver.pwm_autoscale(true);           // necesario para StealthChop

    // Comprueba que el módulo responde por UART
    uint8_t ver = driver.version();
    tmcOk = (ver != 0 && ver != 0xFF);
    Serial.printf("TMC2209 UART: %s (version=0x%02X)\n",
                  tmcOk ? "OK" : "SIN RESPUESTA — revisa R4/R5/PDN_UART", ver);
}
#endif  // USE_UART

// ── Lectura de la IMU + seguridad por inclinación ────────────
void updateIMU() {
    static unsigned long lastImu = 0;
    if (!imuOk || millis() - lastImu < 100) return;   // ~10 Hz
    lastImu = millis();

    int16_t rx, ry, rz;
    if (!adxlReadXYZ(rx, ry, rz)) return;

    // full-res: 256 LSB/g
    float ax = rx / 256.0f, ay = ry / 256.0f, az = rz / 256.0f;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
    float roll  = atan2f(ay, az) * 180.0f / PI;

#if USE_TILT_SAFETY
    tiltLockout = (fabsf(pitch) > MAX_TILT_DEG || fabsf(roll) > MAX_TILT_DEG);
#endif

    static unsigned long lastImuPrint = 0;
    if (millis() - lastImuPrint > 500) {
        lastImuPrint = millis();
        Serial.printf("IMU  pitch:%6.1f°  roll:%6.1f°%s\n",
                      pitch, roll, tiltLockout ? "  ⚠ LOCKOUT" : "");
    }
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 + PS4 + TMC2209(UART) + ADXL345 ===");

    pinMode(PIN_STEP,   OUTPUT);
    pinMode(PIN_DIR,    OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, HIGH);

    // TMC2209
#if USE_UART
    tmcBegin();
#else
    Serial.println("Driver en modo STEP/DIR/EN (sin UART). Ajusta corriente con el pot Vref y microstepping con MS1/MS2.");
#endif

    // IMU por I²C
    Wire.begin(PIN_SDA, PIN_SCL);
#if IMU_INTERNAL_PULLUPS
    pinMode(PIN_SDA, INPUT_PULLUP);   // pull-ups internos (~45k) si no hay R1/R2 externas
    pinMode(PIN_SCL, INPUT_PULLUP);
#endif
    imuOk = adxlBegin();
    Serial.printf("ADXL345 I2C: %s (0x53)\n", imuOk ? "OK" : "NO DETECTADO");

    // Bluepad32
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
    Serial.println("Esperando conexión PS4... (mantén PS + Share)");

    xTaskCreatePinnedToCore(
        stepperTask, "StepperTask", 2048, nullptr, 1, &stepperTaskHandle, 0);
}

// ── Loop principal (Core 1 - BT) ─────────────────────────────
void loop() {
    BP32.update();
    updateIMU();

    if (connectedGamepad == nullptr || !connectedGamepad->isConnected()) {
        delay(10);
        return;
    }

    GamepadPtr gp = connectedGamepad;
    int r2 = gp->throttle();
    int l2 = gp->brake();

    bool r2Active = (r2 > TRIGGER_DEADZONE);
    bool l2Active = (l2 > TRIGGER_DEADZONE);

    if (!r2Active && !l2Active) {
        motorEnabled   = false;
        targetPeriodUs = MAX_STEP_US;
    } else if (r2Active && !l2Active) {
        motorDir       = true;
        targetPeriodUs = triggerToPeriod(r2);
        motorEnabled   = true;
    } else if (l2Active && !r2Active) {
        motorDir       = false;
        targetPeriodUs = triggerToPeriod(l2);
        motorEnabled   = true;
    } else {
        if (r2 >= l2) { motorDir = true;  targetPeriodUs = triggerToPeriod(r2); }
        else          { motorDir = false; targetPeriodUs = triggerToPeriod(l2); }
        motorEnabled = true;
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        lastPrint = millis();
        Serial.printf("R2:%4d L2:%4d Dir:%s Period:%ldus Motor:%s\n",
            r2, l2, motorDir ? "FWD" : "BWD", currentPeriodUs,
            (motorEnabled && !tiltLockout) ? "ON" : "OFF");
    }

    delay(10);
}
