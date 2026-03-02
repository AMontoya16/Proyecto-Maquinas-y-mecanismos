#include <AccelStepper.h>

// Pines del driver del motor a pasos
#define STEP_PIN 2
#define DIR_PIN  4

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Motor de 1.8° por paso → 200 pasos por revolución
const int STEPS_PER_REV = 200;

float gradosObjetivo = 0;

void setup() {

  Serial.begin(115200);

  stepper.setMaxSpeed(700);
  stepper.setAcceleration(1100);

  // Se define la posición inicial del motor como 0 pasos
  stepper.setCurrentPosition(0);

  /*
    IMPORTANTE:
    Este programa usa posiciones ABSOLUTAS mediante moveTo().

    Esto significa que:
    - El ángulo que se ingresa por el puerto serial es una posición objetivo,
      no un desplazamiento relativo.
    - La referencia de 0° es la posición en la que se encuentra el motor
      cuando inicia el programa.
    - Ejemplo:
        45  → el motor va a 45°
        -45 → el motor va a -45° (no regresa a 0°, sino que va a la posición -45°)
  */

  Serial.println("Ingrese un angulo en grados:");
}

void loop() {

  if (Serial.available() > 0) {

    gradosObjetivo = Serial.parseFloat();

    // Conversión de grados a pasos
    long pasosObjetivo = gradosObjetivo * STEPS_PER_REV / 360.0;

    Serial.print("Moviendo a: ");
    Serial.print(gradosObjetivo);
    Serial.print(" grados (");
    Serial.print(pasosObjetivo);
    Serial.println(" pasos)");

    // Movimiento a posición absoluta
    stepper.moveTo(pasosObjetivo);
    stepper.runToPosition();

    Serial.println("Movimiento terminado");
    Serial.println("Ingrese otro angulo:");
  }
}
