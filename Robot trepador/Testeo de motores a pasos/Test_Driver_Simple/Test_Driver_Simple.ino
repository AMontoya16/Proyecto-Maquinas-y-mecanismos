/**
 * Test mínimo del driver TMC2209 — SIN PS4.
 * Habilita el driver (EN bajo = con torque) y gira el NEMA17 lento y continuo.
 *   · Si gira suave  → driver + motor + cableado OK. El problema era EN/Vref, no daño.
 *   · Si solo zumba/vibra y no avanza → revisa Vref (¿en cero?) o cableado de bobinas.
 *   · Si no hace NADA ni sostiene → sube el Vref; si sigue, sospecha del driver.
 *
 * Pines: STEP=5  DIR=18  ENABLE=19 (activo LOW). MS1/MS2 a GND = 1/8.
 * Antes de flashear: CIERRA el Monitor Serie del IDE (tiene el puerto ocupado).
 */
#define PIN_STEP    5
#define PIN_DIR     18
#define PIN_ENABLE  19
#define STEP_US     900   // periodo por micro-paso (más grande = más lento y seguro)

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STEP,   OUTPUT);
  pinMode(PIN_DIR,    OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);    // driver HABILITADO (debe sostener el eje con fuerza)
  digitalWrite(PIN_DIR,    HIGH);   // un sentido fijo
  Serial.println("Test driver: el motor debe girar LENTO y CONTINUO, y resistir al pararlo a mano.");
}

void loop() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(3);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(STEP_US);
}
