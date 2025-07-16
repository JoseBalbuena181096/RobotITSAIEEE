#include "config.h"
#include "obstaculos.h"
#include "motores.h"
#include "sensores_linea.h"

void setupObstaculos() {
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90); // Posición inicial
  distanceReadings.reserve(SERVO_POSITIONS);
}

void agregarDistancia(float distancia) {
  if (distanceReadings.size() < SERVO_POSITIONS) {
    distanceReadings.push_back(distancia);
  } else {
    distanceReadings[distanceReadingIndex] = distancia;
    distanceReadingIndex = (distanceReadingIndex + 1) % SERVO_POSITIONS;
  }
}

void leerDistanciaConServo() {
  servoMotor.write(servoAngles[currentServoPos]);
  delay(100); // Esperar que el servo se mueva
  
  int distance = sonar.ping_cm();
  if (distance == 0) distance = MAX_DISTANCE;
  
  agregarDistancia(distance);
  
  currentServoPos = (currentServoPos + 1) % SERVO_POSITIONS;
}

bool hayObstaculo() {
  if (distanceReadings.empty()) return false;
  
  float suma = 0;
  for (float val : distanceReadings) {
    suma += val;
  }
  
  float promedio = suma / distanceReadings.size();
  return promedio <= OBSTACLE_THRESHOLD;
}

bool buscarLinea() {
  leerSensores();
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i]) {
      return true;
    }
  }
  return false;
}

void esquivarObstaculo() {
  unsigned long tiempoActual = millis();
  
  switch (faseEsquivar) {
    case 0: // Giro a la izquierda por 3 segundos
      if (tiempoActual - tiempoEsquivar < 3000) {
        moverMotores(-150, 150);
      } else {
        faseEsquivar = 1;
        tiempoEsquivar = tiempoActual;
      }
      break;
      
    case 1: // Adelante por 5 segundos
      if (tiempoActual - tiempoEsquivar < 5000) {
        moverMotores(150, 150);
      } else {
        faseEsquivar = 2;
        tiempoEsquivar = tiempoActual;
      }
      break;
      
    case 2: // Giro a la derecha por 3 segundos
      if (tiempoActual - tiempoEsquivar < 3000) {
        moverMotores(150, -150);
      } else {
        faseEsquivar = 3;
        tiempoEsquivar = tiempoActual;
      }
      break;
      
    case 3: // Adelante por 10 segundos
      if (tiempoActual - tiempoEsquivar < 10000) {
        moverMotores(150, 150);
      } else {
        faseEsquivar = 4;
        tiempoEsquivar = tiempoActual;
      }
      break;
      
    case 4: // Giro a la derecha por 3 segundos
      if (tiempoActual - tiempoEsquivar < 3000) {
        moverMotores(150, -150);
      } else {
        estadoActual = BUSCAR_LINEA;
        faseEsquivar = 0;
      }
      break;
  }
}
