# Sistema de Registro y Reproducción de Telemetría (MPU6050 + Tarjeta SD)

<p align="center">
  <img src="https://github.com/ElesMiranda/KiCad/blob/6df3cb1f88e2e9281ef96f8527bdb241f5ff2e4c/Imag/MPUSD.png" alt="Vista previa del Dashboard de Reproducción" width="800">
</p>

Un dashboard profesional en Python para la visualización y análisis de datos inerciales post-vuelo/captura. Este proyecto adquiere datos de una IMU de 6 ejes (MPU6050) a alta frecuencia (100Hz), los almacena localmente en un módulo de memoria MicroSD y permite su posterior reproducción en una interfaz gráfica con modelado 3D interactivo y gráficas temporales.

---

## Arquitectura y Protocolos de Comunicación

Este proyecto abandona la transmisión en vivo por cable para permitir que el hardware sea completamente autónomo. Para lograrlo, utiliza dos protocolos de comunicación distintos operando en simultáneo:

### 1. Comunicación Microcontrolador a Memoria: SPI (Serial Peripheral Interface)

<p align="center">
  <img src="https://github.com/ElesMiranda/KiCad/blob/6df3cb1f88e2e9281ef96f8527bdb241f5ff2e4c/Imag/SPI.png" alt="Arquitectura del Bus SPI" width="500">
</p>

Para lograr escribir datos a alta velocidad en la tarjeta SD sin generar cuellos de botella en el muestreo de 100Hz, utilizamos el protocolo **SPI**. A diferencia del I2C, el SPI es un bus de comunicación síncrona full-duplex (puede enviar y recibir al mismo tiempo) diseñado para altas velocidades de transferencia.

Utiliza 4 líneas principales:
* **MOSI (Master Out Slave In):** Envía comandos y datos desde el Arduino a la SD.
* **MISO (Master In Slave Out):** Recibe respuestas de la SD al Arduino.
* **SCK (Serial Clock):** Sincroniza la transmisión a alta velocidad.
* **CS (Chip Select):** Permite activar o desactivar el módulo SD específico.

> **Optimización de Escritura:** En lugar de abrir y cerrar el archivo en cada ciclo (lo cual es un proceso extremadamente lento), el sistema mantiene el archivo abierto y utiliza un buffer en RAM. Cada 100 lecturas (1 segundo), se ejecuta el comando `flush()` para forzar el guardado físico en la memoria flash de la SD, previniendo la pérdida de datos ante desconexiones de energía.

### 2. Comunicación Hardware a Sensor: I2C (Inter-Integrated Circuit)
El MPU6050 sigue utilizando el protocolo **I2C** (`0x68`) a una velocidad de reloj configurada a 400kHz para asegurar lecturas casi instantáneas de los 6 ejes (Acelerómetro y Giroscopio) y mantener el estricto ciclo de muestreo de 10ms.

### 3. Almacenamiento y Decodificación (CSV)

<p align="center">
  <img src="" alt="Estructura de datos CSV" width="400">
</p>

Los datos se guardan en la memoria SD en un archivo estricto de valores separados por comas (`IMU_DATA.csv`). Cada paquete contiene 6 variables seguidas de un salto de línea (`\n`):
`ax, ay, az, gx, gy, gz\n`

El programa en Python ya no lee el puerto serie, sino que carga este archivo local, simula el tiempo transcurrido (10ms por línea) y aplica los filtros matemáticos (Kalman y Filtro Complementario) para reconstruir el movimiento 3D.

---

## Esquema de Conexión (Hardware)

<p align="center">
  <img src="https://github.com/ElesMiranda/KiCad/blob/b0fef77940569a5d2e06df39aef16d2f21198d1e/Imag/ConexionesSD.jpg" alt="Esquema de Conexión de Hardware" width="600">
</p>

El sistema requiere la integración del MPU6050, el módulo SD y un LED de estado. Las conexiones estándar para Arduino Uno/Nano son las siguientes:

**Módulo MicroSD (Bus SPI)**
| Pin del Módulo | Arduino Uno / Nano | Descripción |
| :--- | :--- | :--- |
| **VCC** | 5V | Alimentación principal |
| **GND** | GND | Tierra común |
| **MISO** | Pin 12 | Línea de recepción de datos |
| **MOSI** | Pin 11 | Línea de transmisión de datos |
| **SCK** | Pin 13 | Reloj del bus SPI |
| **CS** | Pin 4 | Selector de Chip (Chip Select) |

**Sensor MPU6050 (Bus I2C) y Periféricos**
| Componente | Arduino Uno / Nano | Descripción |
| :--- | :--- | :--- |
| **MPU - VCC** | 5V | Alimentación (GY-521 LDO) |
| **MPU - GND** | GND | Tierra común |
| **MPU - SDA** | A4 | Línea de datos I2C |
| **MPU - SCL** | A5 | Reloj I2C |
| **LED Status** | Pin 8 | Indicador de grabación/error |

> **ADVERTENCIA TÉCNICA (Módulo SD):** El módulo SD puede tener picos altos de consumo al momento de escribir en la memoria flash. Asegúrese de proporcionar una fuente de alimentación estable. Además, la tarjeta MicroSD debe estar formateada en FAT16 o FAT32 (preferiblemente de 32GB o menos) para ser reconocida correctamente por la librería estándar de Arduino.

---

## Estructura del Directorio

El repositorio unifica la adquisición y simulación en una sola carpeta principal:

* **`/MPUSD`**: Este directorio contiene tanto el firmware de adquisición en C++ (`MPUSD.ino`) como el software de telemetría en Python (`MPUSD.py`).

---

## Requisitos y Guía de Ejecución (Software)

### 1. Preparar el Data Logger (C++)
1. Abre el código `MPUSD.ino` en tu Arduino IDE. Asegúrate de tener instalada la librería predeterminada `SD` y `Wire`.
2. Conecta tu hardware según la tabla superior.
3. Inserta la tarjeta MicroSD formateada.
4. Sube el código al microcontrolador.
5. Observa el LED: Si parpadea infinitamente, revisa las conexiones o el formato de la SD. Si se enciende de forma sólida, ¡el registro a 100Hz ha comenzado!

### 2. Preparar el Entorno Python
Para visualizar los datos grabados, necesitarás Python 3.7+ instalado en tu computadora. Abre tu terminal e instala las dependencias de renderizado matemático y gráfico:
*(Nota: Ya no se requiere `pyserial` al no haber conexión en vivo).*

```bash
pip install pyqt5 pyqtgraph PyOpenGL numpy
