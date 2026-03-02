/*
   Controlling a stepper with AccelStepper library
   Modified so that 0 is always consistent
   and the motor makes one full turn forward and back
   Con pausa de 2 segundos en cada extremo
*/

#include <AccelStepper.h>

// Definimos STEP en D2 y DIR en D4
#define STEP_PIN 2
#define DIR_PIN  4

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int STEPS_PER_REV = 200;

void setup() {
  stepper.setMaxSpeed(700);
  stepper.setAcceleration(1100);
  stepper.setCurrentPosition(0);
}

void loop() {
  // Giro hacia la izquierda
  stepper.moveTo(-10 * STEPS_PER_REV);
  stepper.runToPosition();
  delay(1000);   // espera 2 segundos

  stepper.setCurrentPosition(0);

  // Giro hacia la derecha
  stepper.moveTo(10 * STEPS_PER_REV);
  stepper.runToPosition();
  delay(1000);   // espera 2 segundos

  stepper.setCurrentPosition(0);
}
