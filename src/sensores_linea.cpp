#include "config.h"
#include "sensores_linea.h"
#include "motores.h"

void setupSensoresLinea() {
  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  positionStack.reserve(STACK_SIZE);
}

void leerSensores() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
}

void agregarAlStack(int valor) {
  if (positionStack.size() < STACK_SIZE) {
    positionStack.push_back(valor);
  } else {
    positionStack[stackIndex] = valor;
    stackIndex = (stackIndex + 1) % STACK_SIZE;
  }
}

int obtenerPromedioStack() {
  if (positionStack.empty()) return 0;
  
  long suma = 0;
  for (int val : positionStack) {
    suma += val;
  }
  
  return suma / positionStack.size();
}

int calcularCentroide() {
  int suma = 0;
  int count = 0;
  
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i]) {
      suma += sensorWeights[i];
      count++;
    }
  }
  
  if (count == 0) {
    return obtenerPromedioStack();
  }
  
  agregarAlStack(suma);
  
  return suma;
}

void controlPD(int centroide) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  
  float error = 0 - centroide;
  float derivative = (error - previousError) / deltaTime;
  
  float output = Kp * error + Kd * derivative;
  
  int baseSpeed = BASE_SPEED;
  int leftSpeed = baseSpeed + output;
  int rightSpeed = baseSpeed - output;
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  if (error >= 20) {
    moverMotores(0, 200);
  } else if (error <= -20) {
    moverMotores(200, 0);
  } else {
    moverMotores(leftSpeed, rightSpeed);
  }
  
  previousError = error;
  previousTime = currentTime;
}
