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

**Diagrama de Posicionamiento de Sensores TCRT5000:**

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

    Dirección de Avance: ↑

    Pesos de Sensores:
    • Izquierda: S0(-5), S1(-3), S2(-1)
    • Centro: Entre S2 y S3 (Centroide = 0)
    • Derecha: S3(+1), S4(+3), S5(+5)

    Configuración de Pines:
    ┌────────┬─────────┬──────────────┐
    │ Sensor │ GPIO    │ Peso         │
    ├────────┼─────────┼──────────────┤
    │ S0     │ GPIO36  │ -5 (Izq)     │
    │ S1     │ GPIO39  │ -3           │
    │ S2     │ GPIO34  │ -1           │
    │ S3     │ GPIO35  │ +1           │
    │ S4     │ GPIO32  │ +3           │
    │ S5     │ GPIO23  │ +5 (Der)     │
    └────────┴─────────┴──────────────┘

    Funcionamiento:
    • Sensor activo (1): Detecta línea negra
    • Sensor inactivo (0): Detecta superficie blanca
    • Centroide = Σ(Peso × Estado) / Σ(Estados activos)
```
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
┌─────────┐   ┌───┐   ┌──────────┐   ┌─────────┐   ┌─────────┐
│ REF=0   │──▶│ Σ │──▶│    PD    │──▶│  ROBOT  │──▶│ SENSOR  │
│(Centro) │   │   │   │ Kp + Kd  │   │ Motores │   │Centroide│
└─────────┘   └─┬─┘   └──────────┘   └─────────┘   └────┬────┘
                ▲                                        │
                │                                        │
                └────────────────────────────────────────┘
                           Retroalimentación

Flujo de Señales:
• Error = Referencia - Posición_Actual
• Corrección = Kp × Error + Kd × (Error - Error_anterior)
• Velocidades = Velocidad_Base ± Corrección

Parámetros de Control:
┌─────────────┬─────────┬──────────────────────┐
│ Parámetro   │ Valor   │ Función              │
├─────────────┼─────────┼──────────────────────┤
│ Kp          │ 15.0    │ Respuesta rápida     │
│ Kd          │ 8.0     │ Estabilidad          │
│ BASE_SPEED  │ 150     │ Velocidad base       │
│ Referencia  │ 0       │ Centro de línea      │
└─────────────┴─────────┴──────────────────────┘
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

**Sistema de Detección Ultrasónica Fija HC-SR04:**

```
                    DETECCIÓN ULTRASÓNICA FIJA

    ┌─────────┐    ┌─────────┐                    ┌─────────┐
    │  ROBOT  │────│ HC-SR04 │)))))))))))))))))))│OBSTÁCULO│
    │         │    │ (Fijo)  │                    │         │
    └─────────┘    └─────────┘                    └─────────┘
                        │                              │
                        │◄─────── 20 cm ──────────────┤
                        │        (Umbral)              │
                        │                              │
                        │◄────── 200 cm ──────────────┤
                        │       (Máximo)               │

    Funcionamiento:
    1. TRIGGER: Envía pulso ultrasónico (10μs)
    2. ECHO: Recibe eco reflejado del obstáculo
    3. Cálculo: Distancia = (Tiempo × Velocidad_Sonido) / 2
    4. Decisión: Si distancia ≤ 20cm → OBSTÁCULO DETECTADO

    Campo de Detección:
                    ╱─────────────╲
                   ╱               ╲
    [ROBOT]───────╱     ZONA DE     ╲
                 ╱     DETECCIÓN     ╲
                ╱      (Fijo)        ╲
               ╱_____________________╲

    Ventajas del Sistema Fijo:
    • Detección inmediata (sin delays de servo)
    • Mayor velocidad de respuesta
    • Menor complejidad de código
    • Menor consumo energético
    • Mayor confiabilidad

    Configuración de Pines:
    ┌─────────┬─────────┬──────────────────┐
    │ Señal   │ GPIO    │ Función          │
    ├─────────┼─────────┼──────────────────┤
    │ TRIGGER │ GPIO5   │ Envío de pulso   │
    │ ECHO    │ GPIO4   │ Recepción de eco │
    └─────────┴─────────┴──────────────────┘

    Parámetros:
    • Umbral de detección: 20 cm
    • Alcance máximo: 200 cm
    • Ángulo de detección: ~15° (campo fijo)
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

```
                    RUTINA DE ESQUIVE POR DERECHA

                        ┌─────────────────┐
                        │   OBSTÁCULO     │
                        │   DETECTADO     │
                        └─────────┬───────┘
                                  │
                                  ▼
                        ┌─────────────────┐
                        │     FASE 0      │
                        │ Giro Izquierda  │
                        │    (3 seg)      │
                        │  (-150, 150)    │
                        └─────────┬───────┘
                                  │
                                  ▼
                        ┌─────────────────┐
                        │     FASE 1      │
                        │    Avanzar      │
                        │    (5 seg)      │
                        │  (150, 150)     │
                        └─────────┬───────┘
                                  │
                                  ▼
                        ┌─────────────────┐
                        │     FASE 2      │
                        │ Giro Derecha    │
                        │    (3 seg)      │
                        │  (150, -150)    │
                        └─────────┬───────┘
                                  │
                                  ▼
                        ┌─────────────────┐
                        │     FASE 3      │
                        │    Avanzar      │
                        │    (3 seg)      │
                        │  (150, 150)     │
                        └─────────┬───────┘
                                  │
                                  ▼
                        ┌─────────────────┐
                        │   BUSCAR        │
                        │   LÍNEA         │
                        └─────────────────┘

    Trayectoria del Robot:
    
    Línea ────────[■]──────── (■ = Obstáculo)
    Original      │
                  │
                  ▼
              ┌───┐
              │ 1 │ Giro izquierda
              └─┬─┘
                │
                ▼
              ┌───┐
              │ 2 │ Avanzar (rodear)
              └─┬─┘
                │
                ▼
              ┌───┐
              │ 3 │ Giro derecha
              └─┬─┘
                │
                ▼
              ┌───┐
              │ 4 │ Avanzar (retornar)
              └─┬─┘
                │
                ▼
    Línea ──────────────────
    Retomada
```

**Parámetros de la Rutina:**

| Fase | Acción | Duración | Motor Izq | Motor Der | Descripción |
|------|--------|----------|-----------|-----------|-------------|
| 0 | Giro Izq | 3 seg | -150 | 150 | Alejarse del obstáculo |
| 1 | Avanzar | 5 seg | 150 | 150 | Rodear el obstáculo |
| 2 | Giro Der | 3 seg | 150 | -150 | Orientarse hacia la línea |
| 3 | Avanzar | 3 seg | 150 | 150 | Retornar a la línea |
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
