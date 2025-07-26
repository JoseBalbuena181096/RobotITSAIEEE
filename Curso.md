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

**Arquitectura del Sistema de Control:**

```
┌─────────────────────────────────────────────────────────────┐
│                Robot Seguidor de Línea                     │
│            Sistema Simplificado con Ultrasonido Fijo       │
└─────────────────────────────────────────────────────────────┘

    Motor Izq        L298N         Motor Der
   ┌─────────┐    ┌─────────┐    ┌─────────┐
   │ Motor   │◄───┤ Driver  │───►│ Motor   │
   │ Izq     │    │ Motores │    │ Der     │
   └─────────┘    └────┬────┘    └─────────┘
                       │
                       ▼
                  ┌─────────┐
                  │  ESP32  │
                  │ Control │
                  └────┬────┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
        ▼              ▼              ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│6 Sensores   │ │  HC-SR04    │ │ Alimentación│
│TCRT5000     │ │ (Fijo)      │ │   7V BAT    │
│Línea Negra  │ │ Obstáculos  │ │             │
└─────────────┘ └─────────────┘ └─────────────┘

Conexiones:
• ESP32 → L298N: ENA(14), IN1(27), IN2(26), ENB(12), IN3(25), IN4(33)
• ESP32 → HC-SR04: TRIG(5), ECHO(4)
• ESP32 → Sensores: S0(36), S1(39), S2(34), S3(35), S4(32), S5(23)
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

**Configuración de Pines ESP32 DevKit V1:**

```
                    ESP32 DevKit V1
                 ┌─────────────────────┐
                 │                     │
    3V3    ●─────┤ 3V3           VIN   ├─────● VIN
    GND    ●─────┤ GND           GND   ├─────● GND
           ●─────┤ GPIO15       GPIO13├─────●
           ●─────┤ GPIO2        GPIO12├─────● ENB (12)
    ECHO   ●─────┤ GPIO4        GPIO14├─────● ENA (14)
           ●─────┤ GPIO16       GPIO27├─────● IN1 (27)
           ●─────┤ GPIO17       GPIO26├─────● IN2 (26)
    TRIG   ●─────┤ GPIO5        GPIO25├─────● IN3 (25)
           ●─────┤ GPIO18       GPIO33├─────● IN4 (33)
           ●─────┤ GPIO19       GPIO32├─────● S4 (32)
           ●─────┤ GPIO21       GPIO35├─────● S3 (35)
           ●─────┤ GPIO22       GPIO34├─────● S2 (34)
    S5     ●─────┤ GPIO23       GPIO39├─────● S1 (39)
           ●─────┤ GND          GPIO36├─────● S0 (36)
                 │                     │
                 └─────────────────────┘

Asignación de Pines:
┌─────────────┬─────────┬──────────────────────┐
│ Función     │ GPIO    │ Descripción          │
├─────────────┼─────────┼──────────────────────┤
│ Ultrasonido │ GPIO4   │ Echo (Entrada)       │
│             │ GPIO5   │ Trigger (Salida)     │
├─────────────┼─────────┼──────────────────────┤
│ L298N       │ GPIO14  │ ENA (Motor Izq PWM)  │
│ Motores     │ GPIO27  │ IN1 (Motor Izq Dir1) │
│             │ GPIO26  │ IN2 (Motor Izq Dir2) │
│             │ GPIO12  │ ENB (Motor Der PWM)  │
│             │ GPIO25  │ IN3 (Motor Der Dir1) │
│             │ GPIO33  │ IN4 (Motor Der Dir2) │
├─────────────┼─────────┼──────────────────────┤
│ Sensores    │ GPIO36  │ S0 (Izquierda)       │
│ de Línea    │ GPIO39  │ S1                   │
│ TCRT5000    │ GPIO34  │ S2                   │
│             │ GPIO35  │ S3                   │
│             │ GPIO32  │ S4                   │
│             │ GPIO23  │ S5 (Derecha)         │
└─────────────┴─────────┴──────────────────────┘
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

<div align="center">

![Diagrama de Sensores](data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNTAwIiBoZWlnaHQ9IjMwMCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KICA8IS0tIEZvbmRvIC0tPgogIDxyZWN0IHdpZHRoPSI1MDAiIGhlaWdodD0iMzAwIiBmaWxsPSIjZjhmOWZhIiBzdHJva2U9IiNkZWUyZTYiIHN0cm9rZS13aWR0aD0iMiIvPgogIAogIDwhLS0gUm9ib3QgKHZpc3RhIHN1cGVyaW9yKSAtLT4KICA8cmVjdCB4PSIxNTAiIHk9IjEwMCIgd2lkdGg9IjIwMCIgaGVpZ2h0PSIxMjAiIGZpbGw9IiM2Yzc1N2QiIHN0cm9rZT0iIzQ5NTA1NyIgc3Ryb2tlLXdpZHRoPSIyIiByeD0iMTAiLz4KICA8dGV4dCB4PSIyNTAiIHk9IjE2NSIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZmlsbD0id2hpdGUiIGZvbnQtZmFtaWx5PSJBcmlhbCIgZm9udC1zaXplPSIxNCIgZm9udC13ZWlnaHQ9ImJvbGQiPlJPQk9UPC90ZXh0PgogIAogIDwhLS0gU2Vuc29yZXMgVENSVDUwMDAgLS0+CiAgPGc+CiAgICA8IS0tIFNlbnNvciAwIC0tPgogICAgPGNpcmNsZSBjeD0iMTcwIiBjeT0iMjQwIiByPSI4IiBmaWxsPSIjMjhhNzQ1IiBzdHJva2U9IiMxZTdlMzQiIHN0cm9rZS13aWR0aD0iMiIvPgogICAgPHRleHQgeD0iMTcwIiB5PSIyNDUiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGZpbGw9IndoaXRlIiBmb250LWZhbWlseT0iQXJpYWwiIGZvbnQtc2l6ZT0iOCIgZm9udC13ZWlnaHQ9ImJvbGQiPlMwPC90ZXh0PgogICAgPHRleHQgeD0iMTcwIiB5PSIyNjAiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGZpbGw9IiMzMzMiIGZvbnQtZmFtaWx5PSJBcmlhbCIgZm9udC1zaXplPSIxMCI+LTU8L3RleHQ+CiAgICAKICAgIDwhLS0gU2Vuc29yIDEgLS0+CiAgICA8Y2lyY2xlIGN4PSIyMDAiIGN5PSIyNDAiIHI9IjgiIGZpbGw9IiMyOGE3NDUiIHN0cm9rZT0iIzFlN2UzNCIgc3Ryb2tlLXdpZHRoPSIyIi8+CiAgICA8dGV4dCB4PSIyMDAiIHk9IjI0NSIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZmlsbD0id2hpdGUiIGZvbnQtZmFtaWx5PSJBcmlhbCIgZm9udC1zaXplPSI4IiBmb250LXdlaWdodD0iYm9sZCI+UzE8L3RleHQ+CiAgICA8dGV4dCB4PSIyMDAiIHk9IjI2MCIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZmlsbD0iIzMzMyIgZm9udC1mYW1pbHk9IkFyaWFsIiBmb250LXNpemU9IjEwIj4tMzwvdGV4dD4KICAgIAogICAgPCEtLSBTZW5zb3IgMiAtLT4KICAgIDxjaXJjbGUgY3g9IjIzMCIgY3k9IjI0MCIgcj0iOCIgZmlsbD0iIzI4YTc0NSIgc3Ryb2tlPSIjMWU3ZTM0IiBzdHJva2Utd2lkdGg9IjIiLz4KICAgIDx0ZXh0IHg9IjIzMCIgeT0iMjQ1IiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSJ3aGl0ZSIgZm9udC1mYW1pbHk9IkFyaWFsIiBmb250LXNpemU9IjgiIGZvbnQtd2VpZ2h0PSJib2xkIj5TMjwvdGV4dD4KICAgIDx0ZXh0IHg9IjIzMCIgeT0iMjYwIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSIjMzMzIiBmb250LWZhbWlseT0iQXJpYWwiIGZvbnQtc2l6ZT0iMTAiPi0xPC90ZXh0PgogICAgCiAgICA8IS0tIFNlbnNvciAzIC0tPgogICAgPGNpcmNsZSBjeD0iMjcwIiBjeT0iMjQwIiByPSI4IiBmaWxsPSIjMjhhNzQ1IiBzdHJva2U9IiMxZTdlMzQiIHN0cm9rZS13aWR0aD0iMiIvPgogICAgPHRleHQgeD0iMjcwIiB5PSIyNDUiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGZpbGw9IndoaXRlIiBmb250LWZhbWlseT0iQXJpYWwiIGZvbnQtc2l6ZT0iOCIgZm9udC13ZWlnaHQ9ImJvbGQiPlMzPC90ZXh0PgogICAgPHRleHQgeD0iMjcwIiB5PSIyNjAiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGZpbGw9IiMzMzMiIGZvbnQtZmFtaWx5PSJBcmlhbCIgZm9udC1zaXplPSIxMCI+KzE8L3RleHQ+CiAgICAKICAgIDwhLS0gU2Vuc29yIDQgLS0+CiAgICA8Y2lyY2xlIGN4PSIzMDAiIGN5PSIyNDAiIHI9IjgiIGZpbGw9IiMyOGE3NDUiIHN0cm9rZT0iIzFlN2UzNCIgc3Ryb2tlLXdpZHRoPSIyIi8+CiAgICA8dGV4dCB4PSIzMDAiIHk9IjI0NSIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZmlsbD0id2hpdGUiIGZvbnQtZmFtaWx5PSJBcmlhbCIgZm9udC1zaXplPSI4IiBmb250LXdlaWdodD0iYm9sZCI+UzQ8L3RleHQ+CiAgICA8dGV4dCB4PSIzMDAiIHk9IjI2MCIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZmlsbD0iIzMzMyIgZm9udC1mYW1pbHk9IkFyaWFsIiBmb250LXNpemU9IjEwIj4rMzwvdGV4dD4KICAgIAogICAgPCEtLSBTZW5zb3IgNSAtLT4KICAgIDxjaXJjbGUgY3g9IjMzMCIgY3k9IjI0MCIgcj0iOCIgZmlsbD0iIzI4YTc0NSIgc3Ryb2tlPSIjMWU3ZTM0IiBzdHJva2Utd2lkdGg9IjIiLz4KICAgIDx0ZXh0IHg9IjMzMCIgeT0iMjQ1IiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSJ3aGl0ZSIgZm9udC1mYW1pbHk9IkFyaWFsIiBmb250LXNpemU9IjgiIGZvbnQtd2VpZ2h0PSJib2xkIj5TNTwvdGV4dD4KICAgIDx0ZXh0IHg9IjMzMCIgeT0iMjYwIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSIjMzMzIiBmb250LWZhbWlseT0iQXJpYWwiIGZvbnQtc2l6ZT0iMTAiPis1PC90ZXh0PgogIDwvZz4KICA8IS0tIEzDrW5lYSBuZWdyYSAtLT4KICA8cmVjdCB4PSIxMDAiIHk9IjI3MCIgd2lkdGg9IjMwMCIgaGVpZ2h0PSI4IiBmaWxsPSIjMjEyNTI5IiByeD0iMiIvPgogIDx0ZXh0IHg9IjI1MCIgeT0iMjkwIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSIjMzMzIiBmb250LWZhbWlseT0iQXJpYWwiIGZvbnQtc2l6ZT0iMTIiIGZvbnQtd2VpZ2h0PSJib2xkIj5Mw61uZWEgTmVncmE8L3RleHQ+Cjwvc3ZnPg==)

</div>

**Alternativa de visualización:**

```
     ROBOT (Vista Superior)
    ┌─────────────────────┐
    │                     │
    │       ROBOT         │
    │                     │
    └─────────────────────┘
         │ │ │   │ │ │
        S0 S1 S2 S3 S4 S5
        -5 -3 -1 +1 +3 +5
    ═══════════════════════
         Línea Negra
```

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

**Diagrama de Bloques del Sistema de Control:**

```
┌─────────┐    ┌───┐    ┌──────────┐    ┌─────────┐    ┌─────────┐
│ REF=0   │───▶│ Σ │───▶│    PD    │───▶│  ROBOT  │───▶│ SENSOR  │
│(Centro) │    │   │    │ Kp + Kd  │    │ Motores │    │Centroide│
└─────────┘    └─┬─┘    └──────────┘    └─────────┘    └────┬────┘
                 ▲                                           │
                 │                                           │
                 └───────────────────────────────────────────┘
                              Retroalimentación

Flujo de Señales:
• Error = Referencia - Posición_Actual
• Corrección = Kp × Error + Kd × (Error - Error_anterior)
• Velocidades = Velocidad_Base ± Corrección
```

**Fórmulas del Control PD:**

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

**Estados del Robot y Transiciones:**

```
                    Máquina de Estados del Robot

                           Sin obstáculo
                        ┌─────────────────┐
                        │                 │
                        ▼                 │
                 ┌─────────────────┐      │
                 │   SEGUIR_LINEA  │──────┘
                 │                 │
                 │ • Control PD    │
                 │ • Verificar     │
                 │   obstáculos    │
                 └─────────┬───────┘
                           │
                           │ Obstáculo detectado
                           │ (distancia ≤ 20cm)
                           ▼
                 ┌─────────────────┐
                 │ESQUIVAR_OBSTACULO│
                 │                 │
                 │ • Giro izquierda│
                 │ • Avanzar       │
                 │ • Giro derecha  │
                 │ • Avanzar       │
                 └─────────┬───────┘
                           │
                           │ Maniobra completada
                           ▼
                 ┌─────────────────┐
                 │  BUSCAR_LINEA   │
                 │                 │
                 │ • Avanzar       │
                 │ • Buscar línea  │
                 │ • Sensores      │
                 └─────────┬───────┘
                           │
                           │ Línea encontrada
                           │ (sensores activos)
                           │
                           └─────────────────┐
                                             │
                                             ▼
                                    ┌─────────────────┐
                                    │   SEGUIR_LINEA  │
                                    │     (Inicio)    │
                                    └─────────────────┘

Condiciones de Transición:
┌─────────────────┬──────────────────┬─────────────────────┐
│ Estado Actual   │ Condición        │ Estado Siguiente    │
├─────────────────┼──────────────────┼─────────────────────┤
│ SEGUIR_LINEA    │ Sin obstáculo    │ SEGUIR_LINEA        │
│ SEGUIR_LINEA    │ Obstáculo ≤ 20cm│ ESQUIVAR_OBSTACULO  │
│ ESQUIVAR_OBST.  │ Maniobra completa│ BUSCAR_LINEA        │
│ BUSCAR_LINEA    │ Línea detectada  │ SEGUIR_LINEA        │
│ BUSCAR_LINEA    │ Sensores activos │ SEGUIR_LINEA        │
└─────────────────┴──────────────────┴─────────────────────┘
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

**Esquema de Conexiones del Robot:**

```
                    ┌─────────────┐
                    │   7V BAT   │ ──────┐
                    └─────────────┘       │
                                          │ (Alimentación)
    ┌────┐     ┌─────────────────┐       │
    │ M1 │◄────┤     L298N       │◄──────┤
    └────┘     │  Driver Motores │       │
               │                 │       │
    ┌────┐     │                 │       │
    │ M2 │◄────┤                 │       │
    └────┘     └─────────┬───────┘       │
                         │               │
                         │ (ENA,IN1,IN2, │
                         │  ENB,IN3,IN4) │
                         ▼               │
               ┌─────────────────┐       │
               │     ESP32       │       │
               │   DevKit V1     │◄──────┘
               │                 │
               │  GPIO Pins:     │
               │  • 5,4: HC-SR04 │
               │  • 36,39,34,35, │
               │    32,23: TCRT  │
               │  • 27,26,25,33: │
               │    L298N        │
               └─────┬───────────┘
                     │
                     ├─────────► ┌─────────────┐
                     │           │   HC-SR04   │
                     │           │ Ultrasonido │
                     │           └─────────────┘
                     │
                     └─────────► ┌─────────────────────┐
                                 │  6x TCRT5000        │
                                 │  Sensores de Línea  │
                                 └─────────────────────┘

Conexiones Específicas:
• ESP32 Pin 27 → L298N ENA
• ESP32 Pin 26 → L298N IN1  
• ESP32 Pin 25 → L298N IN2
• ESP32 Pin 33 → L298N ENB
• ESP32 Pin 32 → L298N IN3
• ESP32 Pin 23 → L298N IN4
• ESP32 Pin 5  → HC-SR04 TRIG
• ESP32 Pin 4  → HC-SR04 ECHO
• ESP32 Pins 36,39,34,35,32,23 → TCRT5000 (S0-S5)
```

### 🔄 Diagrama de Flujo Principal del Programa

**Flujo de Ejecución del Robot:**

```
                    ┌─────────────┐
                    │   INICIO    │
                    └──────┬──────┘
                           │
                           ▼
                    ┌─────────────┐
                    │   SETUP()   │
                    │ • Motores   │
                    │ • Sensores  │
                    │ • Obstáculos│
                    └──────┬──────┘
                           │
                           ▼
              ┌─────────────────────┐
              │   Leer Sensores     │
              │ • leerSensores()    │
              │ • calcularCentroide │
              └──────────┬──────────┘
                         │
                         ▼
                    ┌─────────┐
                    │ Estado? │
                    └────┬────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
 ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
 │ SEGUIR_LINEA│ │ESQUIVAR_OBST│ │BUSCAR_LINEA │
 └──────┬──────┘ └─────────────┘ └─────────────┘
        │
        ▼
   ┌─────────┐
   │Obstáculo│ ──NO──┐
   │   ?     │       │
   └────┬────┘       │
        │SÍ          │
        ▼            ▼
 ┌─────────────┐ ┌─────────────┐
 │ Cambiar a   │ │ Control PD  │
 │ ESQUIVAR_   │ │ • Calcular  │
 │ OBSTACULO   │ │ • Aplicar   │
 └─────────────┘ └──────┬──────┘
                        │
                        ▼
                 ┌─────────────┐
                 │ Delay(50ms) │
                 └──────┬──────┘
                        │
                        └──────────┐
                                   │
                                   ▼
                        ┌─────────────────┐
                        │ Volver al Inicio│
                        │   (Bucle Loop)  │
                        └─────────────────┘

Estados del Robot:
• SEGUIR_LINEA: Sigue la línea usando control PD
• ESQUIVAR_OBSTACULO: Ejecuta maniobra de evasión
• BUSCAR_LINEA: Busca la línea después de esquivar
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
