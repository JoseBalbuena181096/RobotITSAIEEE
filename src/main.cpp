#include "config.h"
#include "motores.h"
#include "sensores_linea.h"
#include "obstaculos.h"

void setup() {
  Serial.begin(115200);
  
  setupMotores();
  setupSensoresLinea();
  setupObstaculos();
  
  previousTime = millis();
  
  Serial.println("Robot iniciado - Modo seguir línea");
}

void loop() {
  leerSensores();
  leerDistanciaConServo();
  
  switch (estadoActual) {
    case SEGUIR_LINEA:
      if (hayObstaculo()) {
        estadoActual = ESQUIVAR_OBSTACULO;
        tiempoEsquivar = millis();
        faseEsquivar = 0;
        Serial.println("Obstáculo detectado - Esquivando");
      } else {
        int centroide = calcularCentroide();
        controlPD(centroide);
        
        Serial.print("Centroide: ");
        Serial.println(centroide);
      }
      break;
      
    case ESQUIVAR_OBSTACULO:
      esquivarObstaculo();
      break;
      
    case BUSCAR_LINEA:
      if (buscarLinea()) {
        estadoActual = SEGUIR_LINEA;
        Serial.println("Línea encontrada - Retomando seguimiento");
      } else {
        moverMotores(100, 100);
      }
      break;
  }
  
  delay(50);
}
