# Curso: Robot Seguidor de Línea con Esquiva Obstáculos
## Duración: 6 Horas | Modalidad: Práctica con ESP32

---

## 📋 Información General

### Requisitos Previos
- ✅ Conocimientos básicos de programación en C++
- ✅ Familiaridad con funciones y estructuras de control
- ✅ Robot ya armado y listo para programar
- ✅ Componentes instalados y conectados

### Objetivos del Curso
Al finalizar este curso, los participantes serán capaces de:
- Configurar el entorno de desarrollo con VS Code y PlatformIO
- Implementar control PD para seguimiento de línea
- Programar detección y esquive de obstáculos
- Depurar y optimizar el comportamiento del robot

---

## 🕐 HORA 1: Configuración del Entorno de Desarrollo

### 1.1 Instalación de Visual Studio Code (15 min)
1. Descargar VS Code desde https://code.visualstudio.com/
2. Instalar con configuración por defecto
3. Abrir VS Code y familiarizarse con la interfaz

### 1.2 Instalación de PlatformIO (20 min)
1. Ir a Extensions (Ctrl+Shift+X)
2. Buscar "PlatformIO IDE"
3. Instalar la extensión oficial
4. Reiniciar VS Code
5. Verificar que aparezca el icono de PlatformIO

### 1.3 Instalación de Gemini Code Assist (15 min)
1. Buscar "Gemini Code Assist" en Extensions
2. Instalar y configurar con cuenta de Google
3. Probar funcionalidad básica con comentarios

### 1.4 Crear Proyecto del Robot (10 min)
```bash
# Crear nuevo proyecto PlatformIO
1. Click en PlatformIO Home
2. New Project
3. Name: robot_seguidor_linea
4. Board: ESP32 Dev Module
5. Framework: Arduino
6. Location: Seleccionar carpeta
```

**Ejercicio Práctico**: Crear la estructura de carpetas y archivos base del proyecto.

---

## 🕑 HORA 2: Configuración Inicial y Pruebas Básicas

### 2.1 Configuración del Proyecto (15 min)
1. Crear estructura de archivos:
   - `src/main.cpp`
   - `src/config.h`
   - `src/config.cpp`
   - `src/motores.h`
   - `src/motores.cpp`

2. Configurar `platformio.ini`:
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    ESP32Servo
    NewPing
    
monitor_speed = 115200
```

### 2.2 Prueba de Conexión ESP32 (15 min)
```cpp
// Código de prueba básico
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Robot - Prueba de conexión");
}

void loop() {
    Serial.println("Sistema funcionando...");
    delay(1000);
}
```

### 2.3 Implementación del Archivo config.h (15 min)
```cpp
#ifndef CONFIG_H
#define CONFIG_H

#include <ESP32Servo.h>
#include <NewPing.h>
#include <vector>

// Definición de pines
#define SERVO_PIN 18
#define TRIG_PIN 5
#define ECHO_PIN 4

// Pines del L298N
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 12
#define IN3 25
#define IN4 33

// Constantes
#define MAX_DISTANCE 200
#define OBSTACLE_THRESHOLD 20
#define BASE_SPEED 150

// Estados de la máquina
enum Estado {
    SEGUIR_LINEA,
    ESQUIVAR_OBSTACULO,
    BUSCAR_LINEA
};

extern Estado estadoActual;
extern Servo servoMotor;
extern NewPing sonar;

#endif
```

### 2.4 Prueba de Motores (15 min)
**Ejercicio**: Implementar funciones básicas de movimiento y probar cada motor individualmente.

---

## 🕒 HORA 3: Implementación del Control de Motores

### 3.1 Desarrollo de motores.cpp (20 min)
```cpp
#include "config.h"
#include "motores.h"

void setupMotores() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    Serial.println("Motores configurados");
}

void moverMotores(int velocidadIzq, int velocidadDer) {
    // Motor izquierdo
    if (velocidadIzq > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        velocidadIzq = -velocidadIzq;
    }
    analogWrite(ENA, velocidadIzq);
    
    // Motor derecho
    if (velocidadDer > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        velocidadDer = -velocidadDer;
    }
    analogWrite(ENB, velocidadDer);
}

void detenerMotores() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
```

### 3.2 Pruebas de Movimiento (25 min)
**Ejercicios Prácticos**:
1. **Prueba 1**: Movimiento hacia adelante 2 segundos
2. **Prueba 2**: Giros izquierda y derecha
3. **Prueba 3**: Movimiento en cuadrado
4. **Prueba 4**: Calibración de velocidades

### 3.3 Depuración y Ajustes (15 min)
- Verificar dirección de rotación
- Ajustar velocidades si es necesario
- Calibrar diferencias entre motores

---

## 🕓 HORA 4: Sensores de Línea y Control PD

### 4.1 Configuración de Sensores TCRT5000 (20 min)
```cpp
// En config.h - agregar
#define SENSOR_0 36
#define SENSOR_1 39
#define SENSOR_2 34
#define SENSOR_3 35
#define SENSOR_4 32
#define SENSOR_5 23

extern int sensorPins[6];
extern int sensorWeights[6];
extern bool sensorValues[6];
```

### 4.2 Implementación de sensores_linea.cpp (25 min)
```cpp
#include "config.h"
#include "sensores_linea.h"

void setupSensoresLinea() {
    for (int i = 0; i < 6; i++) {
        pinMode(sensorPins[i], INPUT);
    }
    Serial.println("Sensores de línea configurados");
}

void leerSensores() {
    for (int i = 0; i < 6; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
    }
}

void mostrarSensores() {
    Serial.print("Sensores: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
    }
    Serial.println();
}
```

### 4.3 Pruebas de Sensores (15 min)
**Ejercicios**:
1. **Prueba 1**: Lectura individual de cada sensor
2. **Prueba 2**: Detección de línea negra
3. **Prueba 3**: Respuesta en diferentes posiciones

---

## 🕔 HORA 5: Implementación del Control PD

### 5.1 Comprensión del Control PD (15 min)
**Teoría**:
- Error = Referencia - Posición actual
- Salida = Kp × Error + Kd × (Error - Error_anterior)
- Aplicación al seguimiento de línea

### 5.2 Implementación del Cálculo de Centroide (20 min)
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
        return obtenerPromedioStack();
    }
    
    agregarAlStack(suma);
    return suma;
}
```

### 5.3 Implementación del Control PD (25 min)
```cpp
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
    
    // Control de histéresis
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
```

**Ejercicios Prácticos**:
1. **Calibración de Kp**: Ajustar respuesta proporcional
2. **Calibración de Kd**: Ajustar respuesta derivativa
3. **Prueba en línea recta**: Verificar seguimiento
4. **Prueba en curvas**: Verificar estabilidad

---

## 🕕 HORA 6: Detección de Obstáculos y Máquina de Estados

### 6.1 Configuración del Sensor Ultrasónico y Servo (15 min)
```cpp
// En config.cpp
Servo servoMotor;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

int servoAngles[8] = {30, 45, 60, 75, 90, 105, 120, 135};
std::vector<float> distanceReadings;
int currentServoPos = 0;
```

### 6.2 Implementación de Detección de Obstáculos (20 min)
```cpp
void setupObstaculos() {
    servoMotor.attach(SERVO_PIN);
    servoMotor.write(90);
    distanceReadings.reserve(8);
    Serial.println("Sistema de obstáculos configurado");
}

void leerDistanciaConServo() {
    servoMotor.write(servoAngles[currentServoPos]);
    delay(100);
    
    int distance = sonar.ping_cm();
    if (distance == 0) distance = MAX_DISTANCE;
    
    agregarDistancia(distance);
    currentServoPos = (currentServoPos + 1) % 8;
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
```

### 6.3 Implementación de la Rutina de Esquive (15 min)
```cpp
void esquivarObstaculo() {
    unsigned long tiempoActual = millis();
    
    switch (faseEsquivar) {
        case 0: // Giro izquierda 3 segundos
            if (tiempoActual - tiempoEsquivar < 3000) {
                moverMotores(-150, 150);
            } else {
                faseEsquivar = 1;
                tiempoEsquivar = tiempoActual;
            }
            break;
            
        case 1: // Adelante 5 segundos
            if (tiempoActual - tiempoEsquivar < 5000) {
                moverMotores(150, 150);
            } else {
                faseEsquivar = 2;
                tiempoEsquivar = tiempoActual;
            }
            break;
            
        // ... continuar con las demás fases
    }
}
```

### 6.4 Integración Final y Máquina de Estados (10 min)
```cpp
void loop() {
    leerSensores();
    leerDistanciaConServo();
    
    switch (estadoActual) {
        case SEGUIR_LINEA:
            if (hayObstaculo()) {
                estadoActual = ESQUIVAR_OBSTACULO;
                tiempoEsquivar = millis();
                faseEsquivar = 0;
            } else {
                int centroide = calcularCentroide();
                controlPD(centroide);
            }
            break;
            
        case ESQUIVAR_OBSTACULO:
            esquivarObstaculo();
            break;
            
        case BUSCAR_LINEA:
            if (buscarLinea()) {
                estadoActual = SEGUIR_LINEA;
            } else {
                moverMotores(100, 100);
            }
            break;
    }
    
    delay(50);
}
```

---

## 🎯 Ejercicios Prácticos Finales

### Ejercicio 1: Calibración Completa (15 min)
1. Ajustar umbral de detección de obstáculos
2. Calibrar parámetros Kp y Kd
3. Optimizar velocidades de esquive

### Ejercicio 2: Prueba Integral (15 min)
1. Probar en circuito con línea y obstáculos
2. Verificar transiciones entre estados
3. Ajustar tiempos de maniobra

### Ejercicio 3: Depuración y Optimización (15 min)
1. Usar Serial Monitor para depuración
2. Identificar y corregir problemas
3. Documentar parámetros finales

---

## 📚 Recursos Adicionales

### Comandos Útiles de PlatformIO
```bash
# Compilar proyecto
pio run

# Subir código
pio run --target upload

# Monitor serial
pio device monitor

# Limpiar build
pio run --target clean
```

### Herramientas de Depuración
- **Serial Monitor**: Monitoreo en tiempo real
- **Gemini Code Assist**: Sugerencias de código
- **Breakpoints**: Pausar ejecución (simulación)

### Parámetros de Configuración Recomendados
```cpp
// Control PD
float Kp = 2.0;        // Respuesta proporcional
float Kd = 1.0;        // Respuesta derivativa
int BASE_SPEED = 150;  // Velocidad base

// Detección
int OBSTACLE_THRESHOLD = 20;  // Distancia de detección
int MAX_DISTANCE = 200;       // Rango máximo sensor
```

---

## ✅ Checklist de Finalización

### Configuración Completada
- [ ] VS Code instalado y configurado
- [ ] PlatformIO funcional
- [ ] Gemini Code Assist activo
- [ ] Proyecto creado correctamente

### Funcionalidades Implementadas
- [ ] Control de motores funcional
- [ ] Sensores de línea calibrados
- [ ] Control PD implementado
- [ ] Detección de obstáculos activa
- [ ] Rutina de esquive funcional
- [ ] Máquina de estados operativa

### Pruebas Realizadas
- [ ] Movimiento básico de motores
- [ ] Seguimiento de línea recta
- [ ] Seguimiento en curvas
- [ ] Detección de obstáculos
- [ ] Esquive completo
- [ ] Retorno a línea

---

## 🔧 Troubleshooting Común

### Problema: ESP32 no se detecta
**Solución**:
1. Verificar cable USB
2. Instalar drivers CP210x
3. Presionar botón BOOT al cargar

### Problema: Motores no giran
**Solución**:
1. Verificar conexiones L298N
2. Comprobar alimentación
3. Revisar lógica de control

### Problema: Sensores no detectan línea
**Solución**:
1. Ajustar altura sensores
2. Calibrar umbral de detección
3