# Documentación del Sistema Robot Seguidor de Línea con Esquiva Obstáculos

## Índice
1. [Introducción](#introducción)
2. [Componentes del Sistema](#componentes-del-sistema)
3. [Arquitectura del Software](#arquitectura-del-software)
4. [Configuración de Hardware](#configuración-de-hardware)
5. [Funcionamiento del Sistema](#funcionamiento-del-sistema)
6. [Algoritmos Implementados](#algoritmos-implementados)
7. [Estructura del Código](#estructura-del-código)
8. [Configuración y Compilación](#configuración-y-compilación)

## Introducción

Este sistema implementa un robot autónomo capaz de seguir líneas negras sobre superficies blancas y esquivar obstáculos automáticamente. El robot utiliza una máquina de estados con tres estados principales: seguir línea, esquivar obstáculo y buscar línea.

## Componentes del Sistema

### Hardware Principal
- **Microcontrolador**: ESP32 Dev Kit V1 (30 pines)
- **Control de Motores**: L298N para controlar 2 motores DC
- **Sensores de Línea**: 6 sensores infrarrojos TCRT5000
- **Servo Motor**: SG90 para barrido del sensor ultrasónico
- **Sensor de Distancia**: HC-SR04 ultrasónico
- **Motores**: 2 motores DC con ruedas

### Distribución de Pines

```cpp
// Servo y Sensor Ultrasónico
#define SERVO_PIN 18
#define TRIG_PIN 5
#define ECHO_PIN 4

// Control L298N
#define ENA 14    // Enable Motor A
#define IN1 27    // Motor A Dir 1
#define IN2 26    // Motor A Dir 2
#define ENB 12    // Enable Motor B
#define IN3 25    // Motor B Dir 1
#define IN4 33    // Motor B Dir 2

// Sensores TCRT5000
#define SENSOR_0 36  // Extremo izquierdo
#define SENSOR_1 39  // Izquierda
#define SENSOR_2 34  // Centro izquierdo
#define SENSOR_3 35  // Centro derecho
#define SENSOR_4 32  // Derecha
#define SENSOR_5 23  // Extremo derecho
```

## Arquitectura del Software

### Máquina de Estados
El sistema opera con tres estados principales:

1. **SEGUIR_LINEA**: Estado por defecto, sigue la línea detectada
2. **ESQUIVAR_OBSTACULO**: Ejecuta rutina de esquive por la derecha
3. **BUSCAR_LINEA**: Busca la línea después de esquivar

### Estructura Modular
El código está organizado en módulos especializados:

- **config.h/cpp**: Configuración global y variables
- **motores.h/cpp**: Control de motores DC
- **sensores_linea.h/cpp**: Lectura de sensores IR y control PD
- **obstaculos.h/cpp**: Detección y esquive de obstáculos
- **main.cpp**: Lógica principal y máquina de estados

## Configuración de Hardware

### Sistema de Sensores de Línea
Los 6 sensores TCRT5000 están distribuidos en línea con pesos específicos:

```cpp
int sensorWeights[6] = {-20, -10, 0, 0, 10, 20};
```

- **Sensores 0 y 5** (extremos): ±20 puntos
- **Sensores 1 y 4** (medios): ±10 puntos  
- **Sensores 2 y 3** (centrales): 0 puntos

### Sistema de Detección de Obstáculos
- **Servo SG90**: Barre 120° en 8 posiciones (cada 15°)
- **Ángulos de barrido**: 30°, 45°, 60°, 75°, 90°, 105°, 120°, 135°
- **Umbral de detección**: 20 cm (configurable)

## Funcionamiento del Sistema

### Seguimiento de Línea

#### Cálculo del Centroide
El centroide se calcula sumando los pesos de todos los sensores que detectan línea:

```cpp
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
    return obtenerPromedioStack(); // Usar promedio si no hay detección
  }
  
  return suma;
}
```

#### Control PD
El sistema utiliza un controlador Proporcional-Derivativo:

- **Error**: `0 - centroide`
- **Kp**: 2.0 (ganancia proporcional)
- **Kd**: 1.0 (ganancia derivativa)
- **Velocidad base**: 150 PWM

#### Manejo de Pérdida de Línea
Cuando no se detecta línea, el sistema:
1. Consulta un stack de las últimas 5 posiciones
2. Calcula el promedio como referencia
3. Aplica control de histéresis para errores extremos (±20)

### Detección de Obstáculos

#### Barrido con Servo
El servo realiza un barrido continuo registrando distancias:

```cpp
void leerDistanciaConServo() {
  servoMotor.write(servoAngles[currentServoPos]);
  delay(100); // Tiempo para movimiento del servo
  
  int distance = sonar.ping_cm();
  if (distance == 0) distance = MAX_DISTANCE;
  
  agregarDistancia(distance);
  currentServoPos = (currentServoPos + 1) % SERVO_POSITIONS;
}
```

#### Decisión de Esquive
Se activa esquive cuando el promedio de las 8 lecturas es ≤ 20 cm.

### Rutina de Esquive por Derecha

La maniobra de esquive tiene 5 fases secuenciales:

1. **Fase 0**: Giro izquierda 3 segundos (-150, 150)
2. **Fase 1**: Avance 5 segundos (150, 150)
3. **Fase 2**: Giro derecha 3 segundos (150, -150)
4. **Fase 3**: Avance 10 segundos (150, 150)
5. **Fase 4**: Giro derecha 3 segundos → Cambio a BUSCAR_LINEA

### Búsqueda de Línea

Después del esquive, el robot:
1. Avanza lentamente (100, 100)
2. Monitorea continuamente los sensores
3. Retorna a SEGUIR_LINEA al detectar cualquier sensor activo

## Algoritmos Implementados

### Stack de Posiciones
Mantiene las últimas 5 posiciones para estabilidad:

```cpp
void agregarAlStack(int valor) {
  if (positionStack.size() < STACK_SIZE) {
    positionStack.push_back(valor);
  } else {
    positionStack[stackIndex] = valor;
    stackIndex = (stackIndex + 1) % STACK_SIZE;
  }
}
```

### Buffer Circular de Distancias
Almacena las últimas 8 lecturas del sensor ultrasónico:

```cpp
void agregarDistancia(float distancia) {
  if (distanceReadings.size() < SERVO_POSITIONS) {
    distanceReadings.push_back(distancia);
  } else {
    distanceReadings[distanceReadingIndex] = distancia;
    distanceReadingIndex = (distanceReadingIndex + 1) % SERVO_POSITIONS;
  }
}
```

### Control PD con Histéresis
Implementa control suave y respuesta rápida para casos extremos:

```cpp
void controlPD(int centroide) {
  float error = 0 - centroide;
  float derivative = (error - previousError) / deltaTime;
  float output = Kp * error + Kd * derivative;
  
  // Histéresis para casos extremos
  if (error >= 20) {
    moverMotores(0, 200);      // Giro rápido derecha
  } else if (error <= -20) {
    moverMotores(200, 0);      // Giro rápido izquierda
  } else {
    // Control PD normal
    moverMotores(baseSpeed + output, baseSpeed - output);
  }
}
```

## Estructura del Código

### Archivos Principales

#### config.h
- Definiciones de pines y constantes
- Declaración de variables globales
- Enumeración de estados

#### motores.cpp
- `setupMotores()`: Inicialización de pines
- `moverMotores(int, int)`: Control de velocidad y dirección
- `detenerMotores()`: Parada completa

#### sensores_linea.cpp
- `leerSensores()`: Lectura digital de sensores IR
- `calcularCentroide()`: Cálculo de posición
- `controlPD()`: Implementación del controlador

#### obstaculos.cpp
- `leerDistanciaConServo()`: Barrido y lectura
- `hayObstaculo()`: Detección de obstáculos
- `esquivarObstaculo()`: Rutina de maniobra
- `buscarLinea()`: Búsqueda post-esquive

#### main.cpp
- `setup()`: Inicialización del sistema
- `loop()`: Máquina de estados principal

### Variables Globales Importantes

```cpp
// Estado del sistema
Estado estadoActual = SEGUIR_LINEA;

// Sensores de línea
int sensorPins[6] = {36, 39, 34, 35, 32, 23};
int sensorWeights[6] = {-20, -10, 0, 0, 10, 20};
bool sensorValues[6];

// Control PD
float Kp = 2.0;
float Kd = 1.0;
float previousError = 0;

// Servo y obstáculos
int servoAngles[8] = {30, 45, 60, 75, 90, 105, 120, 135};
std::vector<float> distanceReadings;

// Esquive
unsigned long tiempoEsquivar = 0;
int faseEsquivar = 0;
```

## Configuración y Compilación

### Dependencias Requeridas
```cpp
#include <ESP32Servo.h>  // Control del servo SG90
#include <NewPing.h>     // Sensor ultrasónico HC-SR04
#include <vector>        // Contenedores STL
```

### Configuración de PlatformIO
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    ESP32Servo
    NewPing
```

### Parámetros Ajustables

```cpp
// Detección de obstáculos
#define OBSTACLE_THRESHOLD 20  // Distancia en cm
#define MAX_DISTANCE 200       // Rango máximo sensor

// Control PD
float Kp = 2.0;  // Ganancia proporcional
float Kd = 1.0;  // Ganancia derivativa

// Velocidades
#define BASE_SPEED 150  // Velocidad base PWM
```

## Consideraciones de Diseño

### Ventajas del Sistema
- **Modularidad**: Código organizado en módulos especializados
- **Robustez**: Manejo de pérdida de línea y obstáculos
- **Eficiencia**: Uso de buffers circulares y algoritmos optimizados
- **Configurabilidad**: Parámetros fácilmente ajustables

### Limitaciones
- **Esquive fijo**: Solo esquiva por la derecha
- **Tiempo de barrido**: Delay de 100ms por posición del servo
- **Dependencia de calibración**: Sensores IR requieren ajuste según superficie

### Posibles Mejoras
- Implementar esquive dinámico (izquierda/derecha)
- Agregar sensor de color para diferentes tipos de línea
- Implementar comunicación inalámbrica para monitoreo
- Optimizar tiempos de barrido del servo

---

*Esta documentación corresponde al sistema de robot seguidor de línea con esquiva de obstáculos implementado en ESP32 con arquitectura modular y control PD.*