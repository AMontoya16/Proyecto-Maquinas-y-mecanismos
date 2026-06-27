/**
 * =============================================================
 *  Control por WiFi (web)  →  TMC2209  →  NEMA 17   +   ADXL345 (IMU)
 *  ESP32  ·  SIN dependencia del control PS4
 * =============================================================
 *  El ESP32 crea su propia red WiFi (Access Point). Te conectás
 *  desde el celular o la laptop y abrís http://192.168.4.1 para
 *  controlar el robot (subir / bajar / parar + slider de velocidad)
 *  y ver la inclinación de la IMU en vivo.
 *
 *  Ventajas frente al PS4:
 *    · No depende de emparejar un mando ni de Bluepad32.
 *    · Cualquier dispositivo con navegador sirve, sin instalar app.
 *    · Failsafe: si el navegador/WiFi se cae, el motor se detiene
 *      solo (heartbeat con timeout). Ver CMD_TIMEOUT_MS.
 *
 *  ── Cableado en PROTOBOARD (directo al ESP32, sin PCB) ──────
 *    TMC2209:  STEP → GPIO 5    DIR → GPIO 18    EN → GPIO 19 (activo LOW)
 *    ADXL345:  SDA  → GPIO 21   SCL → GPIO 22
 *
 *    IMU ADXL345 (módulo GY-291) — conexiones OBLIGATORIAS:
 *      VCC → 3V3 del ESP32       GND → GND
 *      SDA → GPIO 21             SCL → GPIO 22
 *      CS  → 3V3   (¡clave! si queda al aire arranca en SPI y NO responde por I2C)
 *      SDO → GND   (fija la dirección en 0x53; si lo ponés a 3V3 sería 0x1D)
 *
 *  Sin PCB no hay pull-ups externos en SDA/SCL: este sketch activa los
 *  internos del ESP32 (IMU_INTERNAL_PULLUPS) y usa I2C a 100 kHz para que
 *  sea confiable con jumpers de protoboard. Si tu GY-291 ya trae pull-ups,
 *  no pasa nada por tener ambos.
 *
 *  Librerías: solo el core de ESP32 (WiFi.h, WebServer.h, Wire.h).
 *             NO necesita Bluepad32 ni librerías externas.
 *
 *  NOTA: WiFi y Bluetooth comparten la radio del ESP32; por eso
 *  este sketch usa SOLO WiFi y no incluye nada de PS4.
 * =============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ── Red WiFi que crea el ESP32 (Access Point) ────────────────
const char* AP_SSID = "RobotTrepador";
const char* AP_PASS = "trepador";   // mínimo 8 caracteres

// ── Pines stepper (= PCB) ────────────────────────────────────
#define PIN_STEP     5
#define PIN_DIR      18
#define PIN_ENABLE   19

// ── Pines I²C de la IMU ──────────────────────────────────────
#define PIN_SDA      21
#define PIN_SCL      22
#define IMU_INTERNAL_PULLUPS  1   // 1 = pull-ups internos del ESP32 (protoboard sin R externas)

// ── Parámetros del stepper (mismos que el ps4.ino que funciona) ─
#define MIN_STEP_US      200      // período mínimo (máx velocidad)
#define MAX_STEP_US     1800      // período máximo (mín velocidad)
#define ACCEL_STEP_US     10      // rampa de aceleración (suavidad)

// ── Failsafe del control web ─────────────────────────────────
//   Si no llega ningún comando en este tiempo, el motor se para.
//   La página manda un "latido" cada ~150 ms, así que 500 ms = 3 fallos.
#define CMD_TIMEOUT_MS   500

// ── IMU ADXL345 ──────────────────────────────────────────────
#define ADXL345_ADDR         0x53   // SDO a GND → 0x53
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_DATAX0      0x32

// ── Seguridad por inclinación ────────────────────────────────
//   Mide la DESVIACIÓN respecto a la orientación de referencia
//   (la del arranque o la del botón "Calibrar"). Así no se bloquea
//   por trepar vertical, sino solo si el robot se ladea/cae.
#define USE_TILT_SAFETY   1
#define MAX_TILT_DEV_DEG  30.0f     // grados de desviación que disparan el corte

// ── Variables globales ───────────────────────────────────────
WebServer server(80);

volatile bool  motorEnabled    = false;
volatile bool  motorDir        = true;
volatile bool  tiltLockout     = false;   // la pone la seguridad por inclinación
volatile long  targetPeriodUs  = MAX_STEP_US;
volatile long  currentPeriodUs = MAX_STEP_US;

unsigned long  lastCmdMs = 0;             // último comando recibido (failsafe)

// Estado de la IMU (lo lee updateIMU, lo publica el servidor)
bool   imuOk    = false;
float  imuPitch = 0, imuRoll = 0;         // ángulos absolutos
float  refPitch = 0, refRoll = 0;         // referencia para la desviación

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

// Lee la IMU y, opcionalmente, fija la referencia con la lectura actual.
bool readTilt(float &pitch, float &roll) {
    int16_t rx, ry, rz;
    if (!adxlReadXYZ(rx, ry, rz)) return false;
    float ax = rx / 256.0f, ay = ry / 256.0f, az = rz / 256.0f;  // 256 LSB/g
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
    roll  = atan2f(ay, az) * 180.0f / PI;
    return true;
}

// ── Escáner I²C de diagnóstico (corre una vez al arrancar) ───
void i2cScan() {
    Serial.println("Escaneando bus I2C...");
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  · dispositivo en 0x%02X%s\n", a,
                          a == ADXL345_ADDR ? "  <- ADXL345" : "");
            found++;
        }
    }
    if (!found) Serial.println("  (ninguno) -> revisa SDA/SCL, alimentación y CS->3V3");
}

// ── Mapeo de velocidad (slider 0-100) a período ──────────────
long speedToPeriod(int s) {
    s = constrain(s, 0, 100);
    float t = s / 100.0f;
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
        if (stepCounter >= 50) {     // cede CPU periódicamente
            stepCounter = 0;
            vTaskDelay(1);
        }
    }
}

// ── Lectura de la IMU + seguridad por inclinación (~10 Hz) ───
void updateIMU() {
    static unsigned long lastImu = 0;
    if (!imuOk || millis() - lastImu < 100) return;
    lastImu = millis();

    float p, r;
    if (!readTilt(p, r)) return;
    imuPitch = p;
    imuRoll  = r;

#if USE_TILT_SAFETY
    float dp = fabsf(p - refPitch);
    float dr = fabsf(r - refRoll);
    tiltLockout = (dp > MAX_TILT_DEV_DEG || dr > MAX_TILT_DEV_DEG);
#endif
}

// ── Página web (HTML + CSS + JS, todo embebido) ──────────────
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Robot Trepador</title>
<style>
  :root{--bg:#10141a;--fg:#e8eef5;--ac:#2e8bff;--dn:#2b3340;--ok:#27c07a;--err:#ff4d4d}
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent;user-select:none}
  body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--fg);
       display:flex;flex-direction:column;align-items:center;gap:16px;padding:18px}
  h1{font-size:20px;margin:4px 0}
  .card{background:#19202b;border-radius:14px;padding:16px;width:100%;max-width:420px}
  .imu{display:flex;justify-content:space-around;font-variant-numeric:tabular-nums}
  .imu b{display:block;font-size:28px}
  .imu span{font-size:12px;opacity:.7}
  #lock{text-align:center;font-weight:700;padding:8px;border-radius:10px;margin-top:10px}
  .btns{display:flex;flex-direction:column;gap:12px}
  button{border:0;border-radius:14px;color:#fff;font-size:22px;font-weight:700;
         padding:26px;touch-action:none;cursor:pointer}
  .up{background:var(--ac)} .dn{background:#7a4dff} .stop{background:var(--err);font-size:18px;padding:16px}
  .slabel{display:flex;justify-content:space-between;margin:6px 2px;font-size:14px}
  input[type=range]{width:100%;height:34px}
  .badge{font-size:12px;padding:3px 8px;border-radius:8px}
  .on{background:var(--ok)} .off{background:var(--dn)}
</style></head><body>
<h1>🤖 Robot Trepador <span id="net" class="badge off">IMU…</span></h1>

<div class="card imu">
  <div><span>Pitch</span><b id="p">--</b>°</div>
  <div><span>Roll</span><b id="r">--</b>°</div>
</div>
<div id="lock" class="off">— sin datos —</div>

<div class="card">
  <div class="slabel"><span>Velocidad</span><span id="sv">50%</span></div>
  <input id="sp" type="range" min="0" max="100" value="50">
</div>

<div class="card btns">
  <button class="up"   id="up">▲ SUBIR</button>
  <button class="dn"   id="dn">▼ BAJAR</button>
  <button class="stop" id="st">■ PARAR</button>
  <button class="stop" id="cal" style="background:#3a4658">⟳ Calibrar IMU (poner como “recto”)</button>
</div>

<script>
let dir=0, speed=50;
const $=id=>document.getElementById(id);
$('sp').oninput=e=>{speed=+e.target.value;$('sv').textContent=speed+'%';};

function send(){
  fetch('/cmd?d='+dir+'&s='+speed).then(r=>r.json()).then(j=>{
    $('p').textContent=j.p.toFixed(1);
    $('r').textContent=j.r.toFixed(1);
    const net=$('net');
    net.textContent=j.imu?'IMU OK':'IMU ✗';
    net.className='badge '+(j.imu?'on':'off');
    const L=$('lock');
    if(!j.imu){L.textContent='IMU no detectada';L.className='off';}
    else if(j.lock){L.textContent='⚠ INCLINACIÓN — MOTOR BLOQUEADO';L.style.background='var(--err)';}
    else{L.textContent='✓ OK';L.style.background='var(--ok)';}
  }).catch(()=>{});
}
setInterval(send,150);

function hold(btn,d){
  const dn=e=>{e.preventDefault();dir=d;send();};
  const up=e=>{e.preventDefault();dir=0;send();};
  btn.addEventListener('pointerdown',dn);
  btn.addEventListener('pointerup',up);
  btn.addEventListener('pointerleave',up);
  btn.addEventListener('pointercancel',up);
}
hold($('up'),1); hold($('dn'),-1);
$('st').onclick=()=>{dir=0;send();};
$('cal').onclick=()=>fetch('/cal');
document.addEventListener('pointerup',()=>{dir=0;});
</script>
</body></html>)HTML";

// ── Handlers del servidor web ────────────────────────────────
void sendImuJson() {
    char buf[80];
    snprintf(buf, sizeof(buf), "{\"p\":%.1f,\"r\":%.1f,\"lock\":%d,\"imu\":%d}",
             imuPitch, imuRoll, tiltLockout ? 1 : 0, imuOk ? 1 : 0);
    server.send(200, "application/json", buf);
}

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleCmd() {
    if (server.hasArg("d")) {
        int d = server.arg("d").toInt();
        int s = server.hasArg("s") ? server.arg("s").toInt() : 0;
        if (d == 0 || s <= 0) {
            motorEnabled   = false;
            targetPeriodUs = MAX_STEP_US;
        } else {
            motorDir       = (d > 0);
            targetPeriodUs = speedToPeriod(s);
            motorEnabled   = true;
        }
        lastCmdMs = millis();
    }
    sendImuJson();
}

void handleCal() {
    refPitch = imuPitch;     // la orientación actual pasa a ser "recto"
    refRoll  = imuRoll;
    tiltLockout = false;
    Serial.printf("IMU calibrada: refPitch=%.1f refRoll=%.1f\n", refPitch, refRoll);
    sendImuJson();
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 + WiFi web + TMC2209 + ADXL345 ===");

    pinMode(PIN_STEP,   OUTPUT);
    pinMode(PIN_DIR,    OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, HIGH);     // driver deshabilitado al inicio

    // IMU por I²C (protoboard, sin pull-ups externos)
    Wire.begin(PIN_SDA, PIN_SCL);
#if IMU_INTERNAL_PULLUPS
    pinMode(PIN_SDA, INPUT_PULLUP);     // pull-ups internos (~45k) como respaldo
    pinMode(PIN_SCL, INPUT_PULLUP);
#endif
    Wire.setClock(100000);              // 100 kHz: más confiable con jumpers de protoboard
    i2cScan();
    imuOk = adxlBegin();
    Serial.printf("ADXL345: %s\n", imuOk ? "OK (0x53)" : "NO DETECTADO");
    if (imuOk) {
        delay(50);
        readTilt(refPitch, refRoll);    // referencia inicial = orientación de arranque
        imuPitch = refPitch; imuRoll = refRoll;
        Serial.printf("Referencia inicial: pitch=%.1f roll=%.1f\n", refPitch, refRoll);
    }

    // WiFi en modo Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("WiFi AP \"%s\" (clave \"%s\")\n", AP_SSID, AP_PASS);
    Serial.print("Abrí en el navegador:  http://");
    Serial.println(WiFi.softAPIP());    // normalmente 192.168.4.1

    server.on("/",    handleRoot);
    server.on("/cmd", handleCmd);
    server.on("/cal", handleCal);
    server.begin();

    xTaskCreatePinnedToCore(
        stepperTask, "StepperTask", 2048, nullptr, 1, &stepperTaskHandle, 0);
}

// ── Loop principal (Core 1) ──────────────────────────────────
void loop() {
    server.handleClient();
    updateIMU();

    // Failsafe: si dejaron de llegar comandos, parar el motor.
    if (motorEnabled && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
        motorEnabled   = false;
        targetPeriodUs = MAX_STEP_US;
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        Serial.printf("pitch:%6.1f roll:%6.1f  motor:%s  period:%ldus%s\n",
            imuPitch, imuRoll,
            (motorEnabled && !tiltLockout) ? (motorDir ? "FWD" : "BWD") : "OFF",
            currentPeriodUs, tiltLockout ? "  <- LOCKOUT" : "");
    }
}
