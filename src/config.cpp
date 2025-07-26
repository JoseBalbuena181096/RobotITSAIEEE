#include "config.h"

// Variables globales
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

Estado estadoActual = SEGUIR_LINEA;
int sensorPins[6] = {SENSOR_0, SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5};
int sensorWeights[6] = {-20, -10, 0, 0, 10, 20};
bool sensorValues[6];
std::vector<int> positionStack;
int stackIndex = 0;

// Variables de servo eliminadas - ya no se usa barrido

// Variables de control PD
float Kp = 2.0;
float Kd = 1.0;
float previousError = 0;
unsigned long previousTime = 0;

// Variables de tiempo para esquivar
unsigned long tiempoEsquivar = 0;
int faseEsquivar = 0;
