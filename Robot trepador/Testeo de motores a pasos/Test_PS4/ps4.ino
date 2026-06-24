/**
 * =============================================================
 *  PS4 DualShock4  →  TMC2209  →  NEMA 17
 *  ESP32 | Bluepad32 framework
 * =============================================================
 *  Pines:
 *    GPIO  5  → STEP
 *    GPIO 18  → DIR
 *    GPIO 19  → ENABLE  (activo LOW en TMC2209)
 *
 *  Triggers:
 *    R2 (0-255)  → adelante
 *    L2 (0-255)  → atrás
 *    Si ambos activos al mismo tiempo → el mayor gana
 *
 *  Lógica de velocidad:
 *    El valor analógico del trigger (0-255) se mapea a un
 *    período de pulso STEP usando una curva cuadrática para
 *    mejor control a baja velocidad.
 *
 *    Período mínimo (máx velocidad): MIN_STEP_US  (200 µs  → ~2500 steps/s)
 *    Período máximo (mín velocidad): MAX_STEP_US  (5000 µs → ~100 steps/s)
 *    Umbral de activación del trigger: TRIGGER_DEADZONE (10/255)
 * =============================================================
 */

#include <Arduino.h>
#include <Bluepad32.h>

// ── Pines ────────────────────────────────────────────────────
#define PIN_STEP    5
#define PIN_DIR     18
#define PIN_ENABLE  19

// ── Parámetros del stepper ───────────────────────────────────
#define MIN_STEP_US      200
#define MAX_STEP_US     1800
#define TRIGGER_DEADZONE  40
#define ACCEL_STEP_US    10

// ── Variables globales ───────────────────────────────────────
GamepadPtr connectedGamepad = nullptr;

volatile bool  motorEnabled  = false;
volatile bool  motorDir      = true;
volatile long  targetPeriodUs = MAX_STEP_US;
volatile long  currentPeriodUs = MAX_STEP_US;

// Task handle para el stepper (core 0, separado del BT en core 1)
TaskHandle_t stepperTaskHandle = nullptr;

// ── Callbacks Bluepad32 ──────────────────────────────────────
void onConnectedGamepad(GamepadPtr gp) {
    connectedGamepad = gp;
    Serial.println("✅ PS4 conectado");

    // Feedback visual: vibración corta al conectar
    gp->setRumble(0x80, 0x40);
    delay(200);
    gp->setRumble(0, 0);
}

void onDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("❌ PS4 desconectado");
    connectedGamepad = nullptr;

    // Detener motor de forma segura
    motorEnabled  = false;
    digitalWrite(PIN_ENABLE, HIGH);
}

// ── Mapeo de trigger a período (curva cuadrática) ────────────
long triggerToPeriod(int triggerVal) {
    if (triggerVal < TRIGGER_DEADZONE) return MAX_STEP_US;

    // Normalizar 0.0 - 1.0
    float t = (float)(triggerVal - TRIGGER_DEADZONE) / (1023.0f - TRIGGER_DEADZONE);
    t = constrain(t, 0.0f, 1.0f);

    // Curva sqrt: sube rápido al inicio, más sensible en zona baja del trigger
    t = sqrtf(t);

    // Interpolar período
    long period = (long)(MAX_STEP_US - t * (MAX_STEP_US - MIN_STEP_US));
    return constrain(period, MIN_STEP_US, MAX_STEP_US);
}

// ── Tarea del stepper (Core 0) ───────────────────────────────
void stepperTask(void* param) {
    digitalWrite(PIN_ENABLE, HIGH);
    digitalWrite(PIN_STEP, LOW);

    int stepCounter = 0;

    while (true) {
        if (!motorEnabled) {
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

        if (stepCounter >= 50) {
            stepCounter = 0;
            vTaskDelay(1);
        }
    }
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 + PS4 + TMC2209 ===");

    pinMode(PIN_STEP,   OUTPUT);
    pinMode(PIN_DIR,    OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, HIGH);

    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
    Serial.println("Esperando conexión PS4... (mantén PS + Share)");

    xTaskCreatePinnedToCore(
        stepperTask,
        "StepperTask",
        2048,
        nullptr,
        1,
        &stepperTaskHandle,
        0
    );
}

// ── Loop principal (Core 1 - BT) ─────────────────────────────
void loop() {
    BP32.update();

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
        motorEnabled = false;
        targetPeriodUs = MAX_STEP_US;

    } else if (r2Active && !l2Active) {
        motorDir      = true;
        targetPeriodUs = triggerToPeriod(r2);
        motorEnabled  = true;

    } else if (l2Active && !r2Active) {
        motorDir      = false;
        targetPeriodUs = triggerToPeriod(l2);
        motorEnabled  = true;

    } else {
        if (r2 >= l2) {
            motorDir      = true;
            targetPeriodUs = triggerToPeriod(r2);
        } else {
            motorDir      = false;
            targetPeriodUs = triggerToPeriod(l2);
        }
        motorEnabled = true;
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        lastPrint = millis();
        Serial.printf("R2:%3d  L2:%3d  Dir:%s  Period:%ld µs  Motor:%s\n",
            r2, l2,
            motorDir ? "FWD" : "BWD",
            currentPeriodUs,
            motorEnabled ? "ON" : "OFF"
        );
    }

    delay(10);
}