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
#define MIN_STEP_US     200    // período mínimo entre pulsos (máx velocidad)
#define MAX_STEP_US     5000   // período máximo entre pulsos (mín velocidad)
#define TRIGGER_DEADZONE 10    // valor mínimo del trigger para activar motor
#define ACCEL_STEP_US   20     // cuántos µs cambia el período por iteración (aceleración)

// ── Variables globales ───────────────────────────────────────
GamepadPtr connectedGamepad = nullptr;

volatile bool  motorEnabled  = false;
volatile bool  motorDir      = true;   // true = adelante, false = atrás
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
    digitalWrite(PIN_ENABLE, HIGH);  // deshabilitar driver
}

// ── Mapeo de trigger a período (curva cuadrática) ────────────
/**
 * Curva cuadrática: da más resolución a velocidades bajas.
 * trigger: 0-255
 * retorna: período en µs (MAX cuando trigger=0, MIN cuando trigger=255)
 */
long triggerToPeriod(int triggerVal) {
    if (triggerVal < TRIGGER_DEADZONE) return MAX_STEP_US;

    // Normalizar 0.0 - 1.0
    float t = (float)(triggerVal - TRIGGER_DEADZONE) / (255.0f - TRIGGER_DEADZONE);

    // Curva cuadrática: t^2 da más control en zona baja
    t = t * t;

    // Interpolar período (inverso: mayor t = menor período = más rápido)
    long period = (long)(MAX_STEP_US - t * (MAX_STEP_US - MIN_STEP_US));
    return constrain(period, MIN_STEP_US, MAX_STEP_US);
}

// ── Tarea del stepper (Core 0) ───────────────────────────────
/**
 * Corre en Core 0 para no interferir con el stack BT (Core 1).
 * Genera pulsos STEP con el período calculado.
 * Implementa rampa de aceleración suave.
 */
void stepperTask(void* param) {
    digitalWrite(PIN_ENABLE, HIGH);  // empieza deshabilitado
    digitalWrite(PIN_STEP, LOW);

    while (true) {
        if (!motorEnabled) {
            digitalWrite(PIN_ENABLE, HIGH);
            currentPeriodUs = MAX_STEP_US;  // reset período para próximo arranque
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // Habilitar driver
        digitalWrite(PIN_ENABLE, LOW);

        // Dirección
        digitalWrite(PIN_DIR, motorDir ? HIGH : LOW);
        delayMicroseconds(5);  // setup time TMC2209

        // Rampa de aceleración/desaceleración suave
        if (currentPeriodUs > targetPeriodUs) {
            currentPeriodUs -= ACCEL_STEP_US;
            if (currentPeriodUs < targetPeriodUs) currentPeriodUs = targetPeriodUs;
        } else if (currentPeriodUs < targetPeriodUs) {
            currentPeriodUs += ACCEL_STEP_US;
            if (currentPeriodUs > targetPeriodUs) currentPeriodUs = targetPeriodUs;
        }

        // Pulso STEP (mínimo 1µs HIGH según datasheet TMC2209)
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(2);
        digitalWrite(PIN_STEP, LOW);

        // Esperar el resto del período
        long waitUs = currentPeriodUs - 2;
        if (waitUs > 0) delayMicroseconds(waitUs);
    }
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 + PS4 + TMC2209 ===");

    // Pines del stepper
    pinMode(PIN_STEP,   OUTPUT);
    pinMode(PIN_DIR,    OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, HIGH);  // deshabilitado al inicio

    // Bluepad32
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();  // comentar esta línea si quieres pairing persistente
    Serial.println("Esperando conexión PS4... (mantén PS + Share)");

    // Lanzar tarea del stepper en Core 0
    xTaskCreatePinnedToCore(
        stepperTask,        // función
        "StepperTask",      // nombre
        2048,               // stack
        nullptr,            // parámetros
        1,                  // prioridad
        &stepperTaskHandle, // handle
        0                   // Core 0
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

    // Leer triggers (0 - 255)
    int r2 = gp->throttle();  // R2 → adelante
    int l2 = gp->brake();     // L2 → atrás

    bool r2Active = (r2 > TRIGGER_DEADZONE);
    bool l2Active = (l2 > TRIGGER_DEADZONE);

    if (!r2Active && !l2Active) {
        // Sin input → detener
        motorEnabled = false;
        targetPeriodUs = MAX_STEP_US;

    } else if (r2Active && !l2Active) {
        // Solo R2 → adelante
        motorDir      = true;
        targetPeriodUs = triggerToPeriod(r2);
        motorEnabled  = true;

    } else if (l2Active && !r2Active) {
        // Solo L2 → atrás
        motorDir      = false;
        targetPeriodUs = triggerToPeriod(l2);
        motorEnabled  = true;

    } else {
        // Ambos activos → el mayor gana
        if (r2 >= l2) {
            motorDir      = true;
            targetPeriodUs = triggerToPeriod(r2);
        } else {
            motorDir      = false;
            targetPeriodUs = triggerToPeriod(l2);
        }
        motorEnabled = true;
    }

    // Debug por Serial (cada 200ms para no saturar)
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

    delay(10);  // ~100Hz de lectura del gamepad
}