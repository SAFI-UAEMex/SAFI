<p align="center">
  <table border="0" cellpadding="0" cellspacing="0">
    <tr>
      <td align="center" valign="middle">
        <img src="./Pictures/LOGO_SAFI a color.png" width="500">
      </td>
      <td width="20"></td> <td align="center" valign="middle">
        <img src="./Pictures/PotroRockets_New PNG.png" width="500">
      </td>
    </tr>
  </table>
</p>

# 🚀 Proyecto Aeroespacial SAFI (Sistema Autónomo de Vuelo Integrado)

Bienvenido al repositorio maestro del **Capítulo Estudiantil SAFI - UAEMex**. 

Este espacio centraliza el trabajo de ingeniería multidisciplinario de todas las divisiones de nuestro equipo. Nuestro objetivo es documentar, versionar y unificar el ciclo de vida completo de nuestros vehículos aeroespaciales: desde los cálculos conceptuales y simulaciones aerodinámicas, hasta el diseño electrónico, la programación de la computadora de vuelo y la manufactura de las aeroestructuras.

---

## 🎯 Propósito del Repositorio

Para asegurar el control de calidad, la trazabilidad de los errores y la colaboración fluida entre los distintos departamentos de ingeniería, este monorepositorio integra:
* **Códigos y Firmware:** Software de vuelo, algoritmos de telemetría y sistemas de ignición.
* **Memorias de Cálculo:** Justificaciones matemáticas, simulaciones de trayectoria y análisis de esfuerzos estructurales.
* **Diseño CAD/EDA:** Archivos fuente de piezas mecánicas en 3D y diagramas de circuitos impresos (PCBs).
* **Sistemas de Sub-Repositorios:** Módulos independientes de ingeniería vinculados a este núcleo maestro.

---

## ⚙️ Arquitectura del Sistema (Divisiones)

El proyecto está dividido en subsistemas críticos de ingeniería. Cada división mantiene su propia estructura de trabajo y documentación dentro de su directorio:

### 📡 1. Aviónica y Telemetría (`/AVIONICA`)
Desarrollo del "cerebro" del vehículo (Computadora LANIAKEA).
* Diseño electrónico de PCBs (KiCad / Altium).
* Firmware de adquisición de datos (Sensores Inerciales y Atmosféricos).
* Arquitectura de comunicación y protocolos (I2C, SPI, UART).
* *Nota: Esta carpeta opera mediante submódulos de Git (Ver sección "Guía de Clonación").*

### 🛠️ 2. Aeroestructuras (`/AEROESTRUCTURAS`)
Diseño mecánico y análisis de viabilidad del fuselaje y componentes.
* Archivos CAD (SolidWorks, Fusion360 o FreeCAD).
* Simulaciones de Análisis de Elementos Finitos (FEA).
* Planos de manufactura y tolerancia.

### 🔥 3. Propulsión (`/PROPULSION`)
Ingeniería de motores y sistemas de empuje.
* Memorias de cálculo termodinámico y perfiles de empuje.
* Diseño de toberas y carcasas de motor.
* Registros de telemetría de bancos de prueba estáticos.

### 🪂 4. Sistema de Recuperación (`/RECUPERACION`)
Sistemas de despliegue y descenso seguro.
* Cálculos de arrastre (Drag) y dimensionamiento de paracaídas.
* Mecanismos de eyección electromecánica y pirotécnica.
* Lógica de despliegue en el firmware de Aviónica.

---

## 💻 Guía de Clonación (¡Importante!)

Debido a que este proyecto utiliza una arquitectura profesional de **Submódulos de Git** para manejar repositorios independientes (como las librerías de KiCad), un clonado normal no descargará todo el código.

Para obtener el cohete completo con todas sus carpetas, los miembros del equipo deben clonar este repositorio por primera vez usando la bandera `--recurse-submodules`:

```bash
git clone --recurse-submodules [https://github.com/SAFI-UAEMex/SAFI.git](https://github.com/SAFI-UAEMex/SAFI.git)
