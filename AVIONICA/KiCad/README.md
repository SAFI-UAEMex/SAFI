
![image alt](https://github.com/ElesMiranda/KiCad/blob/ebfa5abfa8072a82491f4c47a19140047edfba5e/Imag/KiCad-Logo.svg.png)
# KiCad
# Diseño Electrónico en KiCad - SAFI

Bienvenido al repositorio oficial de recursos y diseño electrónico del **Capítulo Estudiantil SAFI**. 

Este espacio está dedicado a estandarizar, almacenar y documentar nuestros flujos de trabajo utilizando **KiCad EDA**. Nuestro objetivo principal es facilitar la colaboración entre todos los miembros del equipo, mantener una biblioteca unificada de componentes y optimizar nuestro proceso de fabricación interna de placas de circuito impreso (PCBs).

---

## Propósito del Repositorio

Para asegurar la calidad y compatibilidad en todos los proyectos de ingeniería del capítulo, este repositorio centraliza:
- **Bibliotecas Propias:** Símbolos esquemáticos y huellas (footprints) validados para los componentes que utilizamos frecuentemente.
- **Reglas de Diseño (DRC):** Configuraciones estandarizadas para garantizar que cualquier miembro diseñe placas que nuestra máquina CNC pueda fresar sin problemas.
- **Plantillas Base:** Archivos de KiCad pre-configurados para acelerar el desarrollo de nuevos circuitos.

## Flujo de Manufactura del Capítulo

Fomentamos una cultura de "Hazlo tú mismo" (DIY). Nuestro flujo de trabajo oficial para llevar los diseños de la pantalla a la placa de baquelita es:

1. **[KiCad](https://www.kicad.org/):** Creación del diagrama esquemático, ruteo de la PCB y exportación de archivos Gerber.
2. **[FlatCAM 8.991](https://drive.google.com/file/d/1pwMMPqekb8oLhkwjraOCaSqU5r90mv8D/view?usp=drive_link):** Procesamiento de los Gerbers para generar las trayectorias de la herramienta (G-code) aislando las pistas.
3. **[Candle](https://drive.google.com/drive/folders/1Qzr3Ldz9wdj51mxDhc2Xyh02n26Dh03L?usp=drive_link):** Control de la máquina CNC para el fresado de pistas, perforación de agujeros (drill) y corte del contorno de la placa.

## 📁 Estructura del Repositorio

- 📂 `/Bibliotecas_Capitulo` - Archivos `.kicad_sym` (símbolos), `.kicad_mod` (huellas) y modelos 3D (`.step`).
- 📂 `/Plantillas_KiCad` - Proyectos base con los grosores de pista y separaciones correctas para nuestro router CNC.
- 📂 `/Guias_y_Tutoriales` - Documentación paso a paso para los nuevos miembros.

## Cómo Colaborar (Para Miembros)

Si eres parte del equipo de electrónica y vas a contribuir a este repositorio:
1. Sincroniza siempre tu carpeta local antes de abrir KiCad abriendo tu terminal y ejecutando: `git pull origin main`.
2. Si diseñas un símbolo o footprint nuevo, asegúrate de guardarlo en la carpeta `/Bibliotecas_Capitulo` para que el resto del equipo pueda aprovecharlo.
3. Guarda tus avances constantemente y haz commits descriptivos (ej. `git commit -m "Agregada guía de grosores de pista para FlatCAM"`).
---
*Repositorio mantenido por el área de Electrónica del Capítulo. ¡El limite son las estrellas!* 
