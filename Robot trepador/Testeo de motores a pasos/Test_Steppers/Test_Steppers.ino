/*
   Control de motor a pasos con la ayuda de la librería AccelStepper
   Modificado para que el 0 sea siempre consistente
   y el motor hace la cantidad de revoluciones dado en un sentido y de vuelta
   con pausa de 2 segundos entre cada rutina de giro.
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
  // Giro en sentido antihorario
  stepper.moveTo(-10 * STEPS_PER_REV);
  stepper.runToPosition();
  delay(1000);   // espera 2 segundos

  stepper.setCurrentPosition(0);

  // Giro en sentido horario
  stepper.moveTo(10 * STEPS_PER_REV);
  stepper.runToPosition();
  delay(1000);   // espera 2 segundos

  stepper.setCurrentPosition(0);
}
