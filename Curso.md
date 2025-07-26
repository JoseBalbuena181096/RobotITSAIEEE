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
- Programar detección y esquive de obstáculos con ultrasonido fijo
- Depurar y optimizar el comportamiento del robot

### ⚠️ Nota Importante sobre el Sistema de Detección
Este curso ha sido actualizado para usar un **sensor ultrasónico fijo** en lugar del sistema original con servo y barrido. Esta simplificación ofrece:
- **Mayor velocidad**: Detección inmediata sin delays de servo
- **Menor complejidad**: Menos código y componentes
- **Mayor confiabilidad**: Menos partes móviles
- **Menor consumo**: Sin motor servo activo

### 📊 Diagrama General del Sistema

```svg
<svg width="600" height="400" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="600" height="400" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- ESP32 -->
  <rect x="250" y="150" width="100" height="60" fill="#007bff" stroke="#0056b3" stroke-width="2" rx="5"/>
  <text x="300" y="185" text-anchor="middle" fill="white" font-family="Arial" font-size="14" font-weight="bold">ESP32</text>
  
  <!-- Sensores de línea -->
  <rect x="50" y="250" width="120" height="40" fill="#28a745" stroke="#1e7e34" stroke-width="2" rx="3"/>
  <text x="110" y="275" text-anchor="middle" fill="white" font-family="Arial" font-size="12">6 Sensores TCRT5000</text>
  
  <!-- Sensor ultrasónico -->
  <rect x="430" y="250" width="120" height="40" fill="#ffc107" stroke="#e0a800" stroke-width="2" rx="3"/>
  <text x="490" y="275" text-anchor="middle" fill="black" font-family="Arial" font-size="12">HC-SR04 (Fijo)</text>
  
  <!-- Motores -->
  <rect x="100" y="50" width="80" height="40" fill="#dc3545" stroke="#c82333" stroke-width="2" rx="3"/>
  <text x="140" y="75" text-anchor="middle" fill="white" font-family="Arial" font-size="12">Motor Izq</text>
  
  <rect x="420" y="50" width="80" height="40" fill="#dc3545" stroke="#c82333" stroke-width="2" rx="3"/>
  <text x="460" y="75" text-anchor="middle" fill="white" font-family="Arial" font-size="12">Motor Der</text>
  
  <!-- L298N -->
  <rect x="250" y="50" width="100" height="40" fill="#6f42c1" stroke="#5a32a3" stroke-width="2" rx="3"/>
  <text x="300" y="75" text-anchor="middle" fill="white" font-family="Arial" font-size="12">L298N</text>
  
  <!-- Conexiones -->
  <line x1="300" y1="150" x2="300" y2="90" stroke="#333" stroke-width="2"/>
  <line x1="250" y1="70" x2="180" y2="70" stroke="#333" stroke-width="2"/>
  <line x1="350" y1="70" x2="420" y2="70" stroke="#333" stroke-width="2"/>
  <line x1="250" y1="180" x2="170" y2="250" stroke="#333" stroke-width="2"/>
  <line x1="350" y1="180" x2="430" y2="250" stroke="#333" stroke-width="2"/>
  
  <!-- Etiquetas -->
  <text x="300" y="30" text-anchor="middle" fill="#333" font-family="Arial" font-size="16" font-weight="bold">Robot Seguidor de Línea</text>
  <text x="300" y="380" text-anchor="middle" fill="#666" font-family="Arial" font-size="12">Sistema Simplificado con Ultrasonido Fijo</text>
</svg>
```

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

#### 📋 Diagrama de Pines del ESP32

```svg
<svg width="500" height="350" xmlns="http://www.w3.org/2000/svg">
  <!-- ESP32 Board -->
  <rect x="150" y="50" width="200" height="250" fill="#2c3e50" stroke="#34495e" stroke-width="3" rx="10"/>
  <text x="250" y="80" text-anchor="middle" fill="white" font-family="Arial" font-size="16" font-weight="bold">ESP32 DevKit V1</text>
  
  <!-- Pines izquierda -->
  <g fill="#ecf0f1" font-family="Arial" font-size="10">
    <!-- Motores -->
    <circle cx="140" cy="120" r="3" fill="#e74c3c"/>
    <text x="120" y="125" text-anchor="end">ENA (14)</text>
    <circle cx="140" cy="140" r="3" fill="#e74c3c"/>
    <text x="120" y="145" text-anchor="end">IN1 (27)</text>
    <circle cx="140" cy="160" r="3" fill="#e74c3c"/>
    <text x="120" y="165" text-anchor="end">IN2 (26)</text>
    <circle cx="140" cy="180" r="3" fill="#e74c3c"/>
    <text x="120" y="185" text-anchor="end">ENB (12)</text>
    <circle cx="140" cy="200" r="3" fill="#e74c3c"/>
    <text x="120" y="205" text-anchor="end">IN3 (25)</text>
    <circle cx="140" cy="220" r="3" fill="#e74c3c"/>
    <text x="120" y="225" text-anchor="end">IN4 (33)</text>
  </g>
  
  <!-- Pines derecha -->
  <g fill="#ecf0f1" font-family="Arial" font-size="10">
    <!-- Ultrasonido -->
    <circle cx="360" cy="120" r="3" fill="#f39c12"/>
    <text x="380" y="125">TRIG (5)</text>
    <circle cx="360" cy="140" r="3" fill="#f39c12"/>
    <text x="380" y="145">ECHO (4)</text>
    
    <!-- Sensores línea -->
    <circle cx="360" cy="170" r="3" fill="#27ae60"/>
    <text x="380" y="175">S0 (36)</text>
    <circle cx="360" cy="190" r="3" fill="#27ae60"/>
    <text x="380" y="195">S1 (39)</text>
    <circle cx="360" cy="210" r="3" fill="#27ae60"/>
    <text x="380" y="215">S2 (34)</text>
    <circle cx="360" cy="230" r="3" fill="#27ae60"/>
    <text x="380" y="235">S3 (35)</text>
    <circle cx="360" cy="250" r="3" fill="#27ae60"/>
    <text x="380" y="255">S4 (32)</text>
    <circle cx="360" cy="270" r="3" fill="#27ae60"/>
    <text x="380" y="275">S5 (23)</text>
  </g>
  
  <!-- Leyenda -->
  <g font-family="Arial" font-size="12">
    <circle cx="50" cy="320" r="5" fill="#e74c3c"/>
    <text x="65" y="325">Motores (L298N)</text>
    <circle cx="200" cy="320" r="5" fill="#f39c12"/>
    <text x="215" y="325">Ultrasonido</text>
    <circle cx="350" cy="320" r="5" fill="#27ae60"/>
    <text x="365" y="325">Sensores Línea</text>
  </g>
</svg>
```

### 2.3 Implementación del Archivo config.h (15 min)
```cpp
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

#### 📍 Diagrama de Posicionamiento de Sensores

```svg
<svg width="500" height="300" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="500" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- Robot (vista superior) -->
  <rect x="150" y="100" width="200" height="120" fill="#6c757d" stroke="#495057" stroke-width="2" rx="10"/>
  <text x="250" y="165" text-anchor="middle" fill="white" font-family="Arial" font-size="14" font-weight="bold">ROBOT</text>
  
  <!-- Sensores TCRT5000 -->
  <g>
    <!-- Sensor 0 -->
    <circle cx="170" cy="240" r="8" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
    <text x="170" y="245" text-anchor="middle" fill="white" font-family="Arial" font-size="8" font-weight="bold">S0</text>
    <text x="170" y="260" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">-5</text>
    
    <!-- Sensor 1 -->
    <circle cx="200" cy="240" r="8" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
    <text x="200" y="245" text-anchor="middle" fill="white" font-family="Arial" font-size="8" font-weight="bold">S1</text>
    <text x="200" y="260" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">-3</text>
    
    <!-- Sensor 2 -->
    <circle cx="230" cy="240" r="8" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
    <text x="230" y="245" text-anchor="middle" fill="white" font-family="Arial" font-size="8" font-weight="bold">S2</text>
    <text x="230" y="260" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">-1</text>
    
    <!-- Sensor 3 -->
    <circle cx="270" cy="240" r="8" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
    <text x="270" y="245" text-anchor="middle" fill="white" font-family="Arial" font-size="8" font-weight="bold">S3</text>
    <text x="270" y="260" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">+1</text>
    
    <!-- Sensor 4 -->
    <circle cx="300" cy="240" r="8" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
    <text x="300" y="245" text-anchor="middle" fill="white" font-family="Arial" font-size="8" font-weight="bold">S4</text>
    <text x="300" y="260" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">+3</text>
    
    <!-- Sensor 5 -->
    <circle cx="330" cy="240" r="8" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
    <text x="330" y="245" text-anchor="middle" fill="white" font-family="Arial" font-size="8" font-weight="bold">S5</text>
    <text x="330" y="260" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">+5</text>
  </g>
  
  <!-- Línea negra -->
  <rect x="100" y="270" width="300" height="8" fill="#212529" rx="2"/>
  <text x="250" y="290" text-anchor="middle" fill="#333" font-family="Arial" font-size="12" font-weight="bold">Línea Negra</text>
  
  <!-- Flechas de dirección -->
  <defs>
    <marker id="arrowdir" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#007bff"/>
    </marker>
  </defs>
  
  <line x1="250" y1="80" x2="250" y2="50" stroke="#007bff" stroke-width="3" marker-end="url(#arrowdir)"/>
  <text x="250" y="40" text-anchor="middle" fill="#007bff" font-family="Arial" font-size="12" font-weight="bold">Dirección de Avance</text>
  
  <!-- Leyenda de pesos -->
  <text x="50" y="30" fill="#333" font-family="Arial" font-size="14" font-weight="bold">Pesos de Sensores:</text>
  <text x="50" y="50" fill="#666" font-family="Arial" font-size="12">Izquierda: -5, -3, -1</text>
  <text x="50" y="70" fill="#666" font-family="Arial" font-size="12">Derecha: +1, +3, +5</text>
  
  <!-- Centroide = 0 (centro) -->
  <line x1="250" y1="240" x2="250" y2="270" stroke="#dc3545" stroke-width="2" stroke-dasharray="5,5"/>
  <text x="260" y="255" fill="#dc3545" font-family="Arial" font-size="10" font-weight="bold">Centroide = 0</text>
</svg>
```
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

#### 📈 Diagrama del Control PD

```svg
<svg width="600" height="300" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="600" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- Referencia -->
  <circle cx="50" cy="150" r="20" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
  <text x="50" y="155" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">REF</text>
  <text x="50" y="190" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Línea Centro</text>
  
  <!-- Sumador -->
  <circle cx="150" cy="150" r="15" fill="#ffc107" stroke="#e0a800" stroke-width="2"/>
  <text x="150" y="155" text-anchor="middle" fill="black" font-family="Arial" font-size="14" font-weight="bold">Σ</text>
  
  <!-- Controlador PD -->
  <rect x="200" y="120" width="80" height="60" fill="#007bff" stroke="#0056b3" stroke-width="2" rx="5"/>
  <text x="240" y="145" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">PD</text>
  <text x="240" y="160" text-anchor="middle" fill="white" font-family="Arial" font-size="10">Kp + Kd</text>
  
  <!-- Robot -->
  <rect x="350" y="120" width="80" height="60" fill="#6f42c1" stroke="#5a32a3" stroke-width="2" rx="5"/>
  <text x="390" y="145" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">ROBOT</text>
  <text x="390" y="160" text-anchor="middle" fill="white" font-family="Arial" font-size="10">Motores</text>
  
  <!-- Sensor -->
  <rect x="480" y="120" width="80" height="60" fill="#28a745" stroke="#1e7e34" stroke-width="2" rx="5"/>
  <text x="520" y="145" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">SENSOR</text>
  <text x="520" y="160" text-anchor="middle" fill="white" font-family="Arial" font-size="10">Centroide</text>
  
  <!-- Flechas -->
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#333"/>
    </marker>
  </defs>
  
  <line x1="70" y1="150" x2="135" y2="150" stroke="#333" stroke-width="2" marker-end="url(#arrowhead)"/>
  <line x1="165" y1="150" x2="200" y2="150" stroke="#333" stroke-width="2" marker-end="url(#arrowhead)"/>
  <line x1="280" y1="150" x2="350" y2="150" stroke="#333" stroke-width="2" marker-end="url(#arrowhead)"/>
  <line x1="430" y1="150" x2="480" y2="150" stroke="#333" stroke-width="2" marker-end="url(#arrowhead)"/>
  
  <!-- Retroalimentación -->
  <line x1="520" y1="180" x2="520" y2="220" stroke="#dc3545" stroke-width="2"/>
  <line x1="520" y1="220" x2="150" y2="220" stroke="#dc3545" stroke-width="2"/>
  <line x1="150" y1="220" x2="150" y2="165" stroke="#dc3545" stroke-width="2" marker-end="url(#arrowhead)"/>
  
  <!-- Etiquetas -->
  <text x="100" y="140" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Error</text>
  <text x="240" y="110" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Corrección</text>
  <text x="390" y="110" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Movimiento</text>
  <text x="520" y="110" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Posición</text>
  <text x="335" y="240" text-anchor="middle" fill="#dc3545" font-family="Arial" font-size="10">Retroalimentación</text>
  
  <!-- Fórmulas -->
  <text x="300" y="30" text-anchor="middle" fill="#333" font-family="Arial" font-size="14" font-weight="bold">Control PD para Seguimiento de Línea</text>
  <text x="300" y="50" text-anchor="middle" fill="#666" font-family="Arial" font-size="12">Error = Referencia - Posición</text>
  <text x="300" y="70" text-anchor="middle" fill="#666" font-family="Arial" font-size="12">Salida = Kp × Error + Kd × (Error - Error_anterior)</text>
</svg>
```

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

### 6.1 Configuración del Sensor Ultrasónico (15 min)
```cpp
// En config.cpp
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
```

### 6.2 Implementación de Detección de Obstáculos (20 min)

#### 🎯 Diagrama de Detección Ultrasónica

```svg
<svg width="500" height="300" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="500" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- Robot -->
  <rect x="50" y="120" width="80" height="60" fill="#6c757d" stroke="#495057" stroke-width="2" rx="5"/>
  <text x="90" y="155" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">ROBOT</text>
  
  <!-- Sensor HC-SR04 -->
  <rect x="130" y="140" width="30" height="20" fill="#ffc107" stroke="#e0a800" stroke-width="2" rx="2"/>
  <text x="145" y="152" text-anchor="middle" fill="black" font-family="Arial" font-size="8" font-weight="bold">HC-SR04</text>
  
  <!-- Cono de detección -->
  <path d="M 160 150 L 350 100 A 50 50 0 0 1 350 200 Z" fill="#007bff" fill-opacity="0.3" stroke="#0056b3" stroke-width="2"/>
  
  <!-- Líneas de alcance -->
  <line x1="160" y1="150" x2="300" y2="150" stroke="#28a745" stroke-width="2" stroke-dasharray="5,5"/>
  <text x="230" y="140" text-anchor="middle" fill="#28a745" font-family="Arial" font-size="10" font-weight="bold">20 cm (Umbral)</text>
  
  <line x1="160" y1="150" x2="400" y2="150" stroke="#dc3545" stroke-width="2" stroke-dasharray="3,3"/>
  <text x="280" y="170" text-anchor="middle" fill="#dc3545" font-family="Arial" font-size="10" font-weight="bold">200 cm (Máximo)</text>
  
  <!-- Obstáculo -->
  <rect x="280" y="120" width="40" height="60" fill="#6f42c1" stroke="#5a32a3" stroke-width="2" rx="3"/>
  <text x="300" y="155" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">OBSTÁCULO</text>
  
  <!-- Ondas ultrasónicas -->
  <g stroke="#007bff" stroke-width="1" fill="none">
    <path d="M 160 150 Q 200 130 240 150" stroke-dasharray="2,2"/>
    <path d="M 160 150 Q 200 150 240 150" stroke-dasharray="2,2"/>
    <path d="M 160 150 Q 200 170 240 150" stroke-dasharray="2,2"/>
  </g>
  
  <!-- Reflexión -->
  <g stroke="#dc3545" stroke-width="1" fill="none">
    <path d="M 280 150 Q 240 130 200 150" stroke-dasharray="2,2"/>
    <path d="M 280 150 Q 240 150 200 150" stroke-dasharray="2,2"/>
    <path d="M 280 150 Q 240 170 200 150" stroke-dasharray="2,2"/>
  </g>
  
  <!-- Etiquetas -->
  <text x="250" y="30" text-anchor="middle" fill="#333" font-family="Arial" font-size="16" font-weight="bold">Detección Ultrasónica Fija</text>
  <text x="200" y="110" text-anchor="middle" fill="#007bff" font-family="Arial" font-size="10">Ondas emitidas</text>
  <text x="200" y="190" text-anchor="middle" fill="#dc3545" font-family="Arial" font-size="10">Ondas reflejadas</text>
  
  <!-- Zona de detección -->
  <text x="350" y="250" text-anchor="middle" fill="#666" font-family="Arial" font-size="12">Zona de Detección</text>
  <text x="350" y="265" text-anchor="middle" fill="#666" font-family="Arial" font-size="10">(Campo de visión fijo)</text>
  
  <!-- Indicador de distancia -->
  <line x1="160" y1="200" x2="280" y2="200" stroke="#333" stroke-width="1"/>
  <line x1="160" y1="195" x2="160" y2="205" stroke="#333" stroke-width="1"/>
  <line x1="280" y1="195" x2="280" y2="205" stroke="#333" stroke-width="1"/>
  <text x="220" y="220" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Distancia medida</text>
</svg>
```
```cpp
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
```

### 6.3 Implementación de la Rutina de Esquive (15 min)

#### 🔄 Diagrama de Flujo de Esquive

```svg
<svg width="600" height="500" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="600" height="500" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- Inicio -->
  <ellipse cx="300" cy="50" rx="60" ry="25" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
  <text x="300" y="57" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">OBSTÁCULO</text>
  <text x="300" y="72" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">DETECTADO</text>
  
  <!-- Fase 0: Giro Izquierda -->
  <rect x="220" y="100" width="160" height="50" fill="#007bff" stroke="#0056b3" stroke-width="2" rx="5"/>
  <text x="300" y="120" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">FASE 0</text>
  <text x="300" y="135" text-anchor="middle" fill="white" font-family="Arial" font-size="11">Giro Izquierda (3s)</text>
  <text x="300" y="148" text-anchor="middle" fill="white" font-family="Arial" font-size="10">(-150, 150)</text>
  
  <!-- Fase 1: Avanzar -->
  <rect x="220" y="180" width="160" height="50" fill="#6f42c1" stroke="#5a32a3" stroke-width="2" rx="5"/>
  <text x="300" y="200" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">FASE 1</text>
  <text x="300" y="215" text-anchor="middle" fill="white" font-family="Arial" font-size="11">Avanzar (5s)</text>
  <text x="300" y="228" text-anchor="middle" fill="white" font-family="Arial" font-size="10">(150, 150)</text>
  
  <!-- Fase 2: Giro Derecha -->
  <rect x="220" y="260" width="160" height="50" fill="#dc3545" stroke="#c82333" stroke-width="2" rx="5"/>
  <text x="300" y="280" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">FASE 2</text>
  <text x="300" y="295" text-anchor="middle" fill="white" font-family="Arial" font-size="11">Giro Derecha (3s)</text>
  <text x="300" y="308" text-anchor="middle" fill="white" font-family="Arial" font-size="10">(150, -150)</text>
  
  <!-- Fase 3: Avanzar -->
  <rect x="220" y="340" width="160" height="50" fill="#ffc107" stroke="#e0a800" stroke-width="2" rx="5"/>
  <text x="300" y="360" text-anchor="middle" fill="black" font-family="Arial" font-size="12" font-weight="bold">FASE 3</text>
  <text x="300" y="375" text-anchor="middle" fill="black" font-family="Arial" font-size="11">Avanzar (3s)</text>
  <text x="300" y="388" text-anchor="middle" fill="black" font-family="Arial" font-size="10">(150, 150)</text>
  
  <!-- Fin -->
  <ellipse cx="300" cy="440" rx="60" ry="25" fill="#17a2b8" stroke="#138496" stroke-width="2"/>
  <text x="300" y="447" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">BUSCAR</text>
  <text x="300" y="462" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">LÍNEA</text>
  
  <!-- Flechas -->
  <defs>
    <marker id="arrowflow" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#333"/>
    </marker>
  </defs>
  
  <line x1="300" y1="75" x2="300" y2="100" stroke="#333" stroke-width="2" marker-end="url(#arrowflow)"/>
  <line x1="300" y1="150" x2="300" y2="180" stroke="#333" stroke-width="2" marker-end="url(#arrowflow)"/>
  <line x1="300" y1="230" x2="300" y2="260" stroke="#333" stroke-width="2" marker-end="url(#arrowflow)"/>
  <line x1="300" y1="310" x2="300" y2="340" stroke="#333" stroke-width="2" marker-end="url(#arrowflow)"/>
  <line x1="300" y1="390" x2="300" y2="415" stroke="#333" stroke-width="2" marker-end="url(#arrowflow)"/>
  
  <!-- Trayectoria del robot -->
  <g transform="translate(450, 50)">
    <text x="0" y="0" fill="#333" font-family="Arial" font-size="14" font-weight="bold">Trayectoria:</text>
    
    <!-- Línea original -->
    <line x1="0" y1="30" x2="100" y2="30" stroke="#212529" stroke-width="4"/>
    <text x="50" y="50" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Línea Original</text>
    
    <!-- Obstáculo -->
    <rect x="40" y="20" width="20" height="20" fill="#6f42c1" stroke="#5a32a3" stroke-width="2"/>
    
    <!-- Trayectoria de esquive -->
    <path d="M 30 30 Q 20 10 10 30 Q 10 50 30 70 Q 50 70 70 50 Q 70 30 80 30" 
          stroke="#dc3545" stroke-width="2" fill="none" stroke-dasharray="3,3"/>
    
    <!-- Puntos de fase -->
    <circle cx="30" cy="30" r="3" fill="#007bff"/>
    <circle cx="10" cy="30" r="3" fill="#6f42c1"/>
    <circle cx="30" cy="70" r="3" fill="#dc3545"/>
    <circle cx="70" cy="50" r="3" fill="#ffc107"/>
    <circle cx="80" cy="30" r="3" fill="#17a2b8"/>
    
    <text x="0" y="100" fill="#666" font-family="Arial" font-size="9">1. Giro izq</text>
    <text x="0" y="115" fill="#666" font-family="Arial" font-size="9">2. Avanzar</text>
    <text x="0" y="130" fill="#666" font-family="Arial" font-size="9">3. Giro der</text>
    <text x="0" y="145" fill="#666" font-family="Arial" font-size="9">4. Avanzar</text>
  </g>
  
  <!-- Título -->
  <text x="300" y="25" text-anchor="middle" fill="#333" font-family="Arial" font-size="16" font-weight="bold">Rutina de Esquive por Derecha</text>
</svg>
```
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

#### 🔄 Diagrama de Estados del Robot

```svg
<svg width="600" height="400" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="600" height="400" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- Estado: Seguir Línea -->
  <circle cx="150" cy="150" r="60" fill="#28a745" stroke="#1e7e34" stroke-width="3"/>
  <text x="150" y="145" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">SEGUIR</text>
  <text x="150" y="160" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">LÍNEA</text>
  
  <!-- Estado: Esquivar Obstáculo -->
  <circle cx="450" cy="150" r="60" fill="#dc3545" stroke="#c82333" stroke-width="3"/>
  <text x="450" y="145" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">ESQUIVAR</text>
  <text x="450" y="160" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">OBSTÁCULO</text>
  
  <!-- Estado: Buscar Línea -->
  <circle cx="300" cy="300" r="60" fill="#ffc107" stroke="#e0a800" stroke-width="3"/>
  <text x="300" y="295" text-anchor="middle" fill="black" font-family="Arial" font-size="12" font-weight="bold">BUSCAR</text>
  <text x="300" y="310" text-anchor="middle" fill="black" font-family="Arial" font-size="12" font-weight="bold">LÍNEA</text>
  
  <!-- Flechas de transición -->
  <defs>
    <marker id="arrow" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#333"/>
    </marker>
  </defs>
  
  <!-- Seguir -> Esquivar -->
  <path d="M 210 150 Q 330 100 390 150" stroke="#333" stroke-width="3" fill="none" marker-end="url(#arrow)"/>
  <text x="300" y="120" text-anchor="middle" fill="#333" font-family="Arial" font-size="11" font-weight="bold">Obstáculo detectado</text>
  <text x="300" y="135" text-anchor="middle" fill="#666" font-family="Arial" font-size="10">(distancia ≤ 20cm)</text>
  
  <!-- Esquivar -> Buscar -->
  <path d="M 420 200 Q 380 250 340 280" stroke="#333" stroke-width="3" fill="none" marker-end="url(#arrow)"/>
  <text x="400" y="240" text-anchor="middle" fill="#333" font-family="Arial" font-size="11" font-weight="bold">Maniobra</text>
  <text x="400" y="255" text-anchor="middle" fill="#666" font-family="Arial" font-size="10">completada</text>
  
  <!-- Buscar -> Seguir -->
  <path d="M 260 280 Q 200 250 180 200" stroke="#333" stroke-width="3" fill="none" marker-end="url(#arrow)"/>
  <text x="200" y="240" text-anchor="middle" fill="#333" font-family="Arial" font-size="11" font-weight="bold">Línea encontrada</text>
  <text x="200" y="255" text-anchor="middle" fill="#666" font-family="Arial" font-size="10">(sensores activos)</text>
  
  <!-- Bucle en Seguir Línea -->
  <path d="M 90 120 Q 60 90 90 90 Q 120 90 120 120" stroke="#28a745" stroke-width="2" fill="none" marker-end="url(#arrow)"/>
  <text x="90" y="75" text-anchor="middle" fill="#28a745" font-family="Arial" font-size="10">Control PD</text>
  
  <!-- Bucle en Buscar Línea -->
  <path d="M 240 330 Q 210 360 240 360 Q 270 360 270 330" stroke="#ffc107" stroke-width="2" fill="none" marker-end="url(#arrow)"/>
  <text x="240" y="380" text-anchor="middle" fill="#e0a800" font-family="Arial" font-size="10">Avanzar</text>
  
  <!-- Título -->
  <text x="300" y="30" text-anchor="middle" fill="#333" font-family="Arial" font-size="16" font-weight="bold">Máquina de Estados del Robot</text>
  
  <!-- Condiciones iniciales -->
  <circle cx="50" cy="150" r="8" fill="#333"/>
  <line x1="58" y1="150" x2="90" y2="150" stroke="#333" stroke-width="2" marker-end="url(#arrow)"/>
  <text x="50" y="130" text-anchor="middle" fill="#333" font-family="Arial" font-size="10">Inicio</text>
</svg>
```
```cpp
void loop() {
    leerSensores();
    
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

## 📊 Diagramas Adicionales de Referencia

### 🔧 Diagrama de Conexiones Hardware

```svg
<svg width="700" height="500" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="700" height="500" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- ESP32 -->
  <rect x="300" y="200" width="100" height="100" fill="#2c3e50" stroke="#34495e" stroke-width="3" rx="5"/>
  <text x="350" y="240" text-anchor="middle" fill="white" font-family="Arial" font-size="14" font-weight="bold">ESP32</text>
  <text x="350" y="260" text-anchor="middle" fill="white" font-family="Arial" font-size="12">DevKit V1</text>
  
  <!-- L298N -->
  <rect x="100" y="50" width="120" height="80" fill="#6f42c1" stroke="#5a32a3" stroke-width="2" rx="5"/>
  <text x="160" y="85" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">L298N</text>
  <text x="160" y="100" text-anchor="middle" fill="white" font-family="Arial" font-size="10">Driver Motores</text>
  
  <!-- Motores -->
  <circle cx="50" cy="90" r="25" fill="#dc3545" stroke="#c82333" stroke-width="2"/>
  <text x="50" y="95" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">M1</text>
  
  <circle cx="270" cy="90" r="25" fill="#dc3545" stroke="#c82333" stroke-width="2"/>
  <text x="270" y="95" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">M2</text>
  
  <!-- HC-SR04 -->
  <rect x="480" y="150" width="100" height="50" fill="#ffc107" stroke="#e0a800" stroke-width="2" rx="3"/>
  <text x="530" y="170" text-anchor="middle" fill="black" font-family="Arial" font-size="12" font-weight="bold">HC-SR04</text>
  <text x="530" y="185" text-anchor="middle" fill="black" font-family="Arial" font-size="10">Ultrasonido</text>
  
  <!-- Sensores TCRT5000 -->
  <rect x="250" y="350" width="200" height="60" fill="#28a745" stroke="#1e7e34" stroke-width="2" rx="3"/>
  <text x="350" y="375" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">6x TCRT5000</text>
  <text x="350" y="390" text-anchor="middle" fill="white" font-family="Arial" font-size="10">Sensores de Línea</text>
  
  <!-- Conexiones -->
  <g stroke="#333" stroke-width="2" fill="none">
    <!-- ESP32 a L298N -->
    <line x1="300" y1="220" x2="220" y2="120"/>
    <text x="260" y="170" fill="#333" font-family="Arial" font-size="9">ENA,IN1,IN2</text>
    <text x="260" y="180" fill="#333" font-family="Arial" font-size="9">ENB,IN3,IN4</text>
    
    <!-- L298N a Motores -->
    <line x1="100" y1="90" x2="75" y2="90"/>
    <line x1="220" y1="90" x2="245" y2="90"/>
    
    <!-- ESP32 a HC-SR04 -->
    <line x1="400" y1="230" x2="480" y2="180"/>
    <text x="440" y="200" fill="#333" font-family="Arial" font-size="9">TRIG(5)</text>
    <text x="440" y="210" fill="#333" font-family="Arial" font-size="9">ECHO(4)</text>
    
    <!-- ESP32 a Sensores -->
    <line x1="350" y1="300" x2="350" y2="350"/>
    <text x="360" y="325" fill="#333" font-family="Arial" font-size="9">36,39,34,35,32,23</text>
  </g>
  
  <!-- Alimentación -->
  <rect x="550" y="50" width="80" height="60" fill="#fd7e14" stroke="#e8590c" stroke-width="2" rx="3"/>
  <text x="590" y="75" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">12V</text>
  <text x="590" y="90" text-anchor="middle" fill="white" font-family="Arial" font-size="10">Batería</text>
  
  <line x1="550" y1="80" x2="220" y2="80" stroke="#fd7e14" stroke-width="3"/>
  <text x="385" y="70" text-anchor="middle" fill="#fd7e14" font-family="Arial" font-size="10" font-weight="bold">Alimentación Motores</text>
  
  <!-- USB -->
  <rect x="350" y="120" width="40" height="20" fill="#17a2b8" stroke="#138496" stroke-width="2" rx="2"/>
  <text x="370" y="133" text-anchor="middle" fill="white" font-family="Arial" font-size="8" font-weight="bold">USB</text>
  
  <line x1="370" y1="140" x2="370" y2="200" stroke="#17a2b8" stroke-width="2"/>
  <text x="380" y="160" fill="#17a2b8" font-family="Arial" font-size="9">5V ESP32</text>
  
  <!-- Título -->
  <text x="350" y="30" text-anchor="middle" fill="#333" font-family="Arial" font-size="16" font-weight="bold">Diagrama de Conexiones del Robot</text>
  
  <!-- Leyenda -->
  <g transform="translate(50, 400)">
    <text x="0" y="0" fill="#333" font-family="Arial" font-size="12" font-weight="bold">Leyenda:</text>
    <line x1="0" y1="15" x2="20" y2="15" stroke="#333" stroke-width="2"/>
    <text x="25" y="20" fill="#333" font-family="Arial" font-size="10">Señales digitales</text>
    <line x1="0" y1="30" x2="20" y2="30" stroke="#fd7e14" stroke-width="3"/>
    <text x="25" y="35" fill="#333" font-family="Arial" font-size="10">Alimentación</text>
    <line x1="0" y1="45" x2="20" y2="45" stroke="#17a2b8" stroke-width="2"/>
    <text x="25" y="50" fill="#333" font-family="Arial" font-size="10">USB/Programación</text>
  </g>
</svg>
```

### 🔄 Diagrama de Flujo Principal del Programa

```svg
<svg width="400" height="600" xmlns="http://www.w3.org/2000/svg">
  <!-- Fondo -->
  <rect width="400" height="600" fill="#f8f9fa" stroke="#dee2e6" stroke-width="2"/>
  
  <!-- Inicio -->
  <ellipse cx="200" cy="50" rx="50" ry="25" fill="#28a745" stroke="#1e7e34" stroke-width="2"/>
  <text x="200" y="57" text-anchor="middle" fill="white" font-family="Arial" font-size="12" font-weight="bold">INICIO</text>
  
  <!-- Setup -->
  <rect x="150" y="100" width="100" height="40" fill="#007bff" stroke="#0056b3" stroke-width="2" rx="5"/>
  <text x="200" y="125" text-anchor="middle" fill="white" font-family="Arial" font-size="11" font-weight="bold">SETUP()</text>
  
  <!-- Leer Sensores -->
  <rect x="130" y="170" width="140" height="40" fill="#28a745" stroke="#1e7e34" stroke-width="2" rx="5"/>
  <text x="200" y="195" text-anchor="middle" fill="white" font-family="Arial" font-size="11" font-weight="bold">Leer Sensores</text>
  
  <!-- Decisión Estado -->
  <polygon points="200,240 250,270 200,300 150,270" fill="#ffc107" stroke="#e0a800" stroke-width="2"/>
  <text x="200" y="275" text-anchor="middle" fill="black" font-family="Arial" font-size="10" font-weight="bold">Estado?</text>
  
  <!-- Seguir Línea -->
  <rect x="50" y="340" width="100" height="50" fill="#17a2b8" stroke="#138496" stroke-width="2" rx="5"/>
  <text x="100" y="360" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">SEGUIR</text>
  <text x="100" y="375" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">LÍNEA</text>
  
  <!-- Esquivar -->
  <rect x="200" y="340" width="100" height="50" fill="#dc3545" stroke="#c82333" stroke-width="2" rx="5"/>
  <text x="250" y="360" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">ESQUIVAR</text>
  <text x="250" y="375" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">OBSTÁCULO</text>
  
  <!-- Buscar -->
  <rect x="320" y="340" width="70" height="50" fill="#6f42c1" stroke="#5a32a3" stroke-width="2" rx="5"/>
  <text x="355" y="360" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">BUSCAR</text>
  <text x="355" y="375" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">LÍNEA</text>
  
  <!-- Obstáculo? -->
  <polygon points="100,420 130,440 100,460 70,440" fill="#fd7e14" stroke="#e8590c" stroke-width="2"/>
  <text x="100" y="445" text-anchor="middle" fill="black" font-family="Arial" font-size="9" font-weight="bold">Obstáculo?</text>
  
  <!-- Control PD -->
  <rect x="50" y="490" width="100" height="30" fill="#20c997" stroke="#17a085" stroke-width="2" rx="3"/>
  <text x="100" y="510" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">Control PD</text>
  
  <!-- Delay -->
  <rect x="150" y="540" width="100" height="30" fill="#6c757d" stroke="#495057" stroke-width="2" rx="3"/>
  <text x="200" y="560" text-anchor="middle" fill="white" font-family="Arial" font-size="10" font-weight="bold">Delay(50ms)</text>
  
  <!-- Flechas -->
  <defs>
    <marker id="arrowmain" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#333"/>
    </marker>
  </defs>
  
  <line x1="200" y1="75" x2="200" y2="100" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  <line x1="200" y1="140" x2="200" y2="170" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  <line x1="200" y1="210" x2="200" y2="240" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  
  <!-- Ramificaciones -->
  <line x1="170" y1="270" x2="100" y2="340" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  <line x1="200" y1="300" x2="250" y2="340" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  <line x1="230" y1="270" x2="355" y2="340" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  
  <line x1="100" y1="390" x2="100" y2="420" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  <line x1="100" y1="460" x2="100" y2="490" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  
  <!-- Sí/No -->
  <text x="120" y="455" fill="#28a745" font-family="Arial" font-size="9" font-weight="bold">NO</text>
  <text x="160" y="320" fill="#dc3545" font-family="Arial" font-size="9" font-weight="bold">SÍ</text>
  
  <!-- Bucle principal -->
  <line x1="200" y1="570" x2="200" y2="580" stroke="#333" stroke-width="2"/>
  <line x1="200" y1="580" x2="50" y2="580" stroke="#333" stroke-width="2"/>
  <line x1="50" y1="580" x2="50" y2="190" stroke="#333" stroke-width="2"/>
  <line x1="50" y1="190" x2="130" y2="190" stroke="#333" stroke-width="2" marker-end="url(#arrowmain)"/>
  
  <!-- Etiquetas de estado -->
  <text x="80" y="330" fill="#17a2b8" font-family="Arial" font-size="9">SEGUIR_LINEA</text>
  <text x="220" y="330" fill="#dc3545" font-family="Arial" font-size="9">ESQUIVAR_OBSTACULO</text>
  <text x="330" y="330" fill="#6f42c1" font-family="Arial" font-size="9">BUSCAR_LINEA</text>
  
  <!-- Título -->
  <text x="200" y="25" text-anchor="middle" fill="#333" font-family="Arial" font-size="14" font-weight="bold">Flujo Principal del Programa</text>
</svg>
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

// Detección (Ultrasonido fijo)
int OBSTACLE_THRESHOLD = 20;  // Distancia de detección en cm
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
- [ ] Detección de obstáculos con ultrasonido fijo
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
3. Verificar conexión con microcontrolador
