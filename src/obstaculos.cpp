#include "config.h"
#include "obstaculos.h"
#include "motores.h"
#include "sensores_linea.h"

void setupObstaculos() {
  Serial.println("Sistema de obstáculos configurado - Ultrasonido fijo");
}

bool hayObstaculo() {
  int distance = sonar.ping_cm();
  if (distance == 0) distance = MAX_DISTANCE;
  
  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  return distance <= OBSTACLE_THRESHOLD;
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
