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

![Diagrama general del sistema](image.png)

**Conexiones del Sistema:**

| Componente | Pines ESP32            | Función                 |
| ---------- | ---------------------- | ----------------------- |
| L298N      | 14, 27, 26, 12, 25, 33 | Control de motores      |
| HC-SR04    | 5 (TRIG), 4 (ECHO)     | Detección de obstáculos |
| TCRT5000   | 36, 39, 34, 35, 32, 23 | Sensores de línea       |

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

### 2.3 Diagrama de Pines del ESP32

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

**Tabla de Asignación de Pines:**

| Función               | GPIO   | Descripción         |
| --------------------- | ------ | ------------------- |
| **Ultrasonido**       |
| Echo                  | GPIO4  | Entrada de señal    |
| Trigger               | GPIO5  | Salida de pulso     |
| **L298N Motores**     |
| ENA                   | GPIO14 | PWM Motor Izquierdo |
| IN1                   | GPIO27 | Dirección Motor Izq |
| IN2                   | GPIO26 | Dirección Motor Izq |
| ENB                   | GPIO12 | PWM Motor Derecho   |
| IN3                   | GPIO25 | Dirección Motor Der |
| IN4                   | GPIO33 | Dirección Motor Der |
| **Sensores TCRT5000** |
| S0                    | GPIO36 | Sensor Izquierda    |
| S1                    | GPIO39 | Sensor Izq-Centro   |
| S2                    | GPIO34 | Sensor Centro-Izq   |
| S3                    | GPIO35 | Sensor Centro-Der   |
| S4                    | GPIO32 | Sensor Derecha      |
| S5                    | GPIO23 | Sensor Extremo Der  |

### 2.4 Implementación del Archivo config.h (15 min)

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

### 2.5 Prueba de Motores (15 min)

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

### 4.2 Diagrama de Posicionamiento de Sensores

![Diagrama de posicionamiento de sensores](image-1.png)

**Tabla de Configuración de Sensores:**

| Sensor | GPIO | Posición          | Peso | Función                     |
| ------ | ---- | ----------------- | ---- | --------------------------- |
| S0     | 36   | Extremo Izquierdo | -5   | Detección lateral izquierda |
| S1     | 39   | Izquierda         | -3   | Corrección izquierda        |
| S2     | 34   | Centro-Izquierda  | -1   | Centrado fino izquierda     |
| S3     | 35   | Centro-Derecha    | +1   | Centrado fino derecha       |
| S4     | 32   | Derecha           | +3   | Corrección derecha          |
| S5     | 23   | Extremo Derecho   | +5   | Detección lateral derecha   |

**Cálculo del Centroide:**

```
Centroide = Σ(Sensor_i × Peso_i) / Σ(Sensor_i)
```

**Ejemplos de Detección:**

| Sensores Activos | Centroide | Acción                 |
| ---------------- | --------- | ---------------------- |
| S2               | -1        | Robot centrado         |
| S1, S2           | -2        | Giro suave derecha     |
| S2, S3           | 0         | Perfectamente centrado |
| S3, S4           | +2        | Giro suave izquierda   |
| S0               | -5        | Giro fuerte derecha    |
| S5               | +5        | Giro fuerte izquierda  |

### 4.3 Implementación de sensores_linea.cpp (25 min)

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

### 4.4 Pruebas de Sensores (15 min)

**Ejercicios**:

1. **Prueba 1**: Lectura individual de cada sensor
2. **Prueba 2**: Detección de línea negra
3. **Prueba 3**: Respuesta en diferentes posiciones

---

## 🕔 HORA 5: Implementación del Control PD

### 5.1 Diagrama del Sistema de Control PD

![Diagrama del sistema de control PD](image-2.png)

**Parámetros del Control PD:**

| Parámetro  | Valor | Función                            |
| ---------- | ----- | ---------------------------------- |
| Kp         | 15.0  | Respuesta proporcional (velocidad) |
| Kd         | 8.0   | Respuesta derivativa (estabilidad) |
| BASE_SPEED | 150   | Velocidad base de avance           |
| Referencia | 0     | Centro de la línea                 |

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

### 6.2 Diagrama del Sistema de Detección Ultrasónica

![Diagrama del sistema de detección ultrasónica](image-3.png)

**Ventajas del Sistema Fijo:**

- ✅ Detección inmediata (sin delays de servo)
- ✅ Mayor velocidad de respuesta
- ✅ Menor complejidad de código
- ✅ Menor consumo energético
- ✅ Mayor confiabilidad

### 6.3 Implementación de Detección de Obstáculos (20 min)

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

### 6.4 Diagrama de la Rutina de Esquive

![Diagrama de la rutina de esquive](image-4.png)

**Tabla de Parámetros de Esquive:**

| Fase | Acción   | Duración | Motor Izq | Motor Der | Descripción               |
| ---- | -------- | -------- | --------- | --------- | ------------------------- |
| 0    | Giro Izq | 3 seg    | -150      | 150       | Alejarse del obstáculo    |
| 1    | Avanzar  | 5 seg    | 150       | 150       | Rodear el obstáculo       |
| 2    | Giro Der | 3 seg    | 150       | -150      | Orientarse hacia la línea |
| 3    | Avanzar  | 3 seg    | 150       | 150       | Retornar a la línea       |

### 6.5 Implementación de la Rutina de Esquive (15 min)

```cpp
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
```

### 6.6 Máquina de Estados del Robot

![Máquina de Estados del Robot](image-7.png)

**Condiciones de Transición:**

| Estado Actual      | Condición         | Estado Siguiente   |
| ------------------ | ----------------- | ------------------ |
| SEGUIR_LINEA       | Sin obstáculo     | SEGUIR_LINEA       |
| SEGUIR_LINEA       | Obstáculo ≤ 20cm  | ESQUIVAR_OBSTACULO |
| ESQUIVAR_OBSTACULO | Maniobra completa | BUSCAR_LINEA       |
| BUSCAR_LINEA       | Línea detectada   | SEGUIR_LINEA       |

### 6.7 Integración Final y Máquina de Estados (10 min)

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

## 📊 Diagramas de Referencia Adicionales

### 🔧 Diagrama de Conexiones Hardware

![Diagrama de Conexiones Hardware](image-5.png)

### 🔄 Diagrama de Flujo Principal

![Diagrama de Flujo Principal](image-6.png)

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
