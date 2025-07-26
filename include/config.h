#ifndef CONFIG_H
#define CONFIG_H

#include <NewPing.h>
#include <vector>

// Definición de pines
#define TRIG_PIN 5
#define ECHO_PIN 4

// Pines del L298N
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 12
#define IN3 25
#define IN4 33

// Pines sensores TCRT5000
#define SENSOR_0 36
#define SENSOR_1 39
#define SENSOR_2 34
#define SENSOR_3 35
#define SENSOR_4 32
#define SENSOR_5 23

// Constantes
#define MAX_DISTANCE 200
#define OBSTACLE_THRESHOLD 20
#define STACK_SIZE 5
#define BASE_SPEED 150

// Estados de la máquina
enum Estado {
  SEGUIR_LINEA,
  ESQUIVAR_OBSTACULO,
  BUSCAR_LINEA
};

// Variables globales
extern NewPing sonar;

extern Estado estadoActual;
extern int sensorPins[6];
extern int sensorWeights[6];
extern bool sensorValues[6];
extern std::vector<int> positionStack;
extern int stackIndex;

// Variables eliminadas: servo y barrido ya no son necesarios

// Variables de control PD
extern float Kp;
extern float Kd;
extern float previousError;
extern unsigned long previousTime;

// Variables de tiempo para esquivar
extern unsigned long tiempoEsquivar;
extern int faseEsquivar;

#endif // CONFIG_H
