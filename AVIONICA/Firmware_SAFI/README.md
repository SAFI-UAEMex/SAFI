<h1 align="center">SISTEMAS DE AVIÓNICA Y COMUNICACIONES</h1>
<h3 align="center">Proyecto LANIAKEA | Capítulo SAFI</h3>

<p align="center">
  <img src="https://github.com/ElesMiranda/KiCad/blob/3a6db8ffc411503813be159581c3d66bf0790fb9/Imag/Captura%20de%20pantalla%202026-03-13%20231730.png" alt="Banner de Firmware SAFI" width="800">
</p>

> **Elesvan Miranda:**
> ¡Bienvenidos, equipo! He estructurado este directorio de programación con el objetivo de centralizar todos nuestros códigos fuente, librerías y diagramas de conexión. Aquí documentaremos el cerebro de nuestra computadora de vuelo para asegurar lecturas precisas y telemetría en tiempo real. Entender la teoría detrás de estos protocolos es fundamental para dejar de adivinar por qué un sensor no funciona y empezar a diagnosticar como verdaderos ingenieros.
> 
> **Apogeo Objetivo:** `3,000 metros`

---

## Arquitectura de Microcontroladores

Nuestra computadora de vuelo distribuye el procesamiento para evitar cuellos de botella durante la fase de ascenso y recuperación. 

Selecciona el núcleo para acceder a sus códigos de inicialización:

### ![ESP32](https://img.shields.io/badge/Computadora_Central-ESP32-black?style=for-the-badge&logo=espressif)
**[Directorio de Código ESP32]([PON_AQUI_EL_ENLACE_A_LA_CARPETA_DEL_ESP32])** | [Documentación Oficial](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
<br>Encargada del procesamiento pesado, la aplicación de filtros matemáticos y el empaquetado del tren de telemetría.
<p align="center">
  <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/ESP32.jpg" alt="ESP32" width="200">
</p>

### ![Arduino Nano](https://img.shields.io/badge/Nodos_Satélite-Arduino_Nano-00979D?style=for-the-badge&logo=arduino)
**[Directorio de Código Nano]([PON_AQUI_EL_ENLACE_A_LA_CARPETA_DEL_NANO])** | [Documentación Oficial](https://docs.arduino.cc/hardware/nano)
<br>Sistema de redundancia y manejo directo de actuadores (conmutación de transistores, ignición, sistemas de despliegue).
<p align="center">
  <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/ARDUINONANO.jpg" alt="Arduino Nano" width="200">
</p>

### ![Raspberry Pi Pico](https://img.shields.io/badge/Adquisición_de_Datos-RPi_Pico-C51A4A?style=for-the-badge&logo=raspberrypi)
**[Directorio de Código Pico]([PON_AQUI_EL_ENLACE_A_LA_CARPETA_DE_LA_PICO])** | [Documentación Oficial](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html)
<br>Manejo de entradas/salidas de alta velocidad y temporización de precisión.
<p align="center">
  <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/raspberry.jpg" alt="Raspberry Pi Pico" width="200">
</p>

---

## Fundamentos de Comunicación Digital

Los protocolos de comunicación son las normativas que permiten que dos o más dispositivos intercambien datos de manera ordenada. Sin ellos, los pulsos eléctricos serían solo ruido. En aviónica, clasificamos estos buses bajo tres criterios principales:

1. **Sincronización:** * **Síncronos:** Utilizan una línea de "Reloj" (`SCK` o `SCL`). El maestro dicta el ritmo exacto de lectura (SPI, I2C).
   * **Asíncronos:** No hay reloj externo. Ambos dispositivos acuerdan una velocidad previa en baudios (UART).
2. **Dirección de flujo:** * **Half-Duplex:** Ambos hablan, pero no al mismo tiempo, similar a un radio transmisor (I2C, RS-485).
   * **Full-Duplex:** Transmisión y recepción simultánea (SPI, UART).
3. **Topología:** Punto a Punto (UART) o Bus Multidireccional Maestro-Esclavo (SPI, I2C).

---

## Protocolos de Comunicación y Periféricos LANIAKEA

> **ADVERTENCIA DE INTEGRACIÓN:** Verifique estrictamente los niveles lógicos de voltaje (`3.3V` vs `5V`) antes de interconectar módulos. Ignorar esto resultará en daño permanente al silicio.

### 1. Bus I2C (Inter-Integrated Circuit)
**Origen:** Inventado en 1982 por Philips Semiconductor para reducir la cantidad de cables en televisores, permitiendo conectar múltiples dispositivos en una misma placa usando solo 2 cables.
<br>**Clasificación:** Serial, Síncrono, Half-Duplex, Bus Multidireccional.

**Anatomía de la capa física:**
* `SDA` (Serial Data): Única línea por donde viaja la información de ida y vuelta.
* `SCL` (Serial Clock): Señal de sincronización dictada por el maestro.
* **Direccionamiento:** I2C no usa pines habilitadores físicos. El maestro envía un paquete hexadecimal (ej. `0x68`); solo el esclavo con esa dirección de fábrica responde.
* **Resistencias Pull-Up:** La arquitectura interna es *Open-Drain*. Los chips pueden llevar el voltaje a `0V`, pero requieren resistencias físicas conectadas a VCC para retornar al estado "alto". Sin ellas, la línea flota y la comunicación falla.

<p align="center">
  <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/I2C.jpg" alt="Diagrama I2C" width="400">
</p>

**Módulos Integrados:**
* **IMU MPU6050:** Orientación espacial, aceleración y giroscopio.
  <p align="center">
    <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/MPU6050.jpg" alt="MPU6050" width="100">
  </p>
* **Barómetro MS5611:** Altímetro de precisión para detección de apogeo.
  <p align="center">
    <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/MS5611.jpg" alt="MS5611" width="100">
  </p>

**[> Consultar librerías e implementaciones I2C]([PON_AQUI_EL_ENLACE_A_LA_CARPETA_I2C])**

---

### 2. Bus UART (Universal Asynchronous Receiver-Transmitter)
**Origen:** El protocolo más antiguo, derivado de los teletipos de 1960. Diseñado para transmitir texto a largas distancias donde una señal de reloj se degradaría.
<br>**Clasificación:** Serial, Asíncrono, Full-Duplex, Punto a Punto.

**Anatomía de la capa física:**
* `TX` (Transmit): Pin de salida de datos.
* `RX` (Receive): Pin de entrada de datos.
* **Cruce de Líneas:** Exige conexión cruzada. El `TX` del sensor debe ir al `RX` del microcontrolador. Conectar TX con TX causa colisiones de voltaje.
* **Baud Rate:** Al carecer de reloj, la información se agrupa en tramas (con bits de inicio y parada). Emisor y receptor deben estar rigurosamente configurados a la misma velocidad (ej. `9600` o `115200` bps).

<p align="center">
  <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/UART.png" alt="Diagrama UART" width="400">
</p>

**Módulos Integrados:**
* **Radio XBee Pro S2C:** Enlace de telemetría de largo alcance hacia la Estación Terrena.
  <p align="center">
    <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/Radio%20XBee%20Pro%20S2C.jpg" alt="Radio XBee Pro S2C" width="100">
  </p>
* **GPS RYS352A:** Adquisición de coordenadas para recuperación.
  <p align="center">
    <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/GPS%20RYS352A.webp" alt="GPS RYS352A" width="100">
  </p>

**[> Consultar algoritmos de telemetría UART]([PON_AQUI_EL_ENLACE_A_LA_CARPETA_UART])**

---

### 3. Bus SPI (Serial Peripheral Interface)
**Origen:** Desarrollado por Motorola en los 80s para lograr velocidades de transferencia casi paralelas utilizando únicamente 4 cables, optimizando el espacio en silicio.
<br>**Clasificación:** Serial, Síncrono, Full-Duplex, Maestro-Esclavo.

**Anatomía de la capa física:**
* `MOSI` (Master Out Slave In) / `COPI`: Línea de instrucción del microcontrolador al periférico.
* `MISO` (Master In Slave Out) / `CIPO`: Línea de respuesta del periférico.
* `SCK` (Serial Clock): Pulso cuadrado generado por el maestro que sincroniza el tráfico.
* `CS` o `SS` (Chip Select): El maestro posee un pin CS dedicado por periférico. Al poner este pin en estado BAJO (`0V`), el periférico despierta y procesa el bus.
* `RST` (Reset): Auxiliar externo común en módulos SPI de alta velocidad para forzar un reinicio de hardware en caso de bloqueo.

<p align="center">
  <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/SPI.png" alt="Diagrama SPI" width="400">
</p>

**Módulos Integrados:**
* **Módulo Adaptador MicroSD:** Datalogger primario ("Caja Negra") para registro masivo de variables a alta frecuencia.
  <p align="center">
    <img src="https://raw.githubusercontent.com/ElesMiranda/KiCad/main/Imag/SD.jpg" alt="Módulo MicroSD" width="100">
  </p>

**[> Consultar rutinas de escritura del Datalogger SPI]([PON_AQUI_EL_ENLACE_A_LA_CARPETA_SPI])**

---

## Resumen de Integración

| Protocolo | Pines Mínimos | Arquitectura | Velocidad | Ventaja Principal |
| :--- | :--- | :--- | :--- | :--- |
| **SPI** | 4 (`MOSI`, `MISO`, `SCK`, `CS`) | Maestro-Esclavo | Muy Alta (MHz) | Transferencia masiva (Full-Duplex) |
| **I2C** | 2 (`SDA`, `SCL`) | Multidireccional | Media (kHz) | Minimiza uso de pines |
| **UART** | 2 (`TX`, `RX`) | Punto a Punto | Baja (Baudios) | Simplicidad extrema (Asíncrono) |

---

## Directivas de Contribución para el Equipo

Para mantener la integridad del repositorio, todo miembro del Capítulo debe adherirse al siguiente protocolo antes de realizar un `push` a la rama principal:

1. **Sincronización Previa:** Ejecutar `git pull origin main` en el entorno local antes de iniciar cualquier modificación.
2. **Mapeo de Hardware:** Todo archivo fuente (`.ino`, `.cpp`, `.h`) debe incluir en su cabecera un bloque de comentarios detallando los pines físicos exactos utilizados durante las pruebas de validación.
3. **Gestión de Dependencias:** Si un periférico requiere bibliotecas externas, el enlace directo al repositorio oficial debe adjuntarse en la documentación del código.
4. **Control de Compilados:** Este repositorio opera bajo un escudo `.gitignore`. Queda estrictamente prohibido forzar la subida de archivos ejecutables, binarios o archivos temporales de compilación.

<br>
<p align="center">
  <i>Ad Astra!</i>
</p>
