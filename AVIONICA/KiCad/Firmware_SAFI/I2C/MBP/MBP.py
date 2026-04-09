"""
BMP180 Barometric & Altimeter Dashboard
Professional GUI with Real-time Graphs, Relative Altitude Tracking and Auto-Scroll
"""

import sys
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QComboBox, 
                             QGroupBox, QSplitter, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import serial
import serial.tools.list_ports

class BMP180Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BMP180 Sensor Telemetry - Mission Control")
        self.setGeometry(50, 50, 1400, 900)
        
        # Estructuras de datos (últimos 500 puntos)
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.temp_data = deque(maxlen=self.max_points)
        self.press_data = deque(maxlen=self.max_points)
        self.alt_data = deque(maxlen=self.max_points)
        self.time_counter = 0
        
        # Variables para Altitud Relativa (Tara)
        self.reference_altitude = 0.0
        self.use_relative_altitude = False
        
        # Conexión Serial
        self.serial_port = None
        self.is_connected = False
        self.serial_buffer = ""
        
        # Configurar UI
        self.setup_ui()
        self.apply_dark_theme()
        
        # Timer de lectura
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial_data)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        # Panel de Control Superior
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)
        
        # Splitter Principal (Displays Izquierda, Gráficas Derecha)
        splitter = QSplitter(Qt.Horizontal)
        
        # --- COLUMNA IZQUIERDA: Monitores Digitales ---
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        self.lbl_temp = self.create_digital_display("TEMPERATURA", "0.00 °C", "#ff0055")
        self.lbl_press = self.create_digital_display("PRESIÓN ATMOSFÉRICA", "0 Pa", "#00eaff")
        self.lbl_alt = self.create_digital_display("ALTITUD", "0.00 m", "#ffb300")
        
        left_layout.addWidget(self.lbl_temp)
        left_layout.addWidget(self.lbl_press)
        left_layout.addWidget(self.lbl_alt)
        left_layout.addStretch()
        left_widget.setLayout(left_layout)
        
        # --- COLUMNA DERECHA: Gráficas ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Crear Gráficas
        self.plot_temp, self.line_temp = self.create_graph("Evolución de Temperatura (°C)", "#ff0055")
        self.plot_press, self.line_press = self.create_graph("Variación de Presión (Pa)", "#00eaff")
        self.plot_alt, self.line_alt = self.create_graph("Perfil de Elevación (m)", "#ffb300")
        
        right_layout.addWidget(self.plot_temp)
        right_layout.addWidget(self.plot_press)
        right_layout.addWidget(self.plot_alt)
        right_widget.setLayout(right_layout)
        
        # Añadir al splitter
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([400, 1000]) # Proporción de tamaño
        
        main_layout.addWidget(splitter, stretch=1)

    def create_control_panel(self):
        panel = QGroupBox("Panel de Operaciones BMP180")
        panel.setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; color: #aaa; border: 1.5px solid #444; border-radius: 6px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }")
        layout = QHBoxLayout(panel)
        
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.port_combo.setMinimumHeight(35)
        self.refresh_ports()
        
        self.connect_btn = QPushButton("Conectar Sensor")
        self.connect_btn.setMinimumHeight(35)
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        self.tare_btn = QPushButton("📍 Establecer Altitud Cero")
        self.tare_btn.setMinimumHeight(35)
        self.tare_btn.setStyleSheet("background-color: #ffb300; color: #181828; font-weight: bold;")
        self.tare_btn.clicked.connect(self.tare_altitude)
        
        layout.addWidget(QLabel("<b>Puerto Serial:</b>"))
        layout.addWidget(self.port_combo)
        layout.addWidget(self.connect_btn)
        layout.addSpacing(30)
        layout.addWidget(self.tare_btn)
        layout.addStretch(1)
        
        return panel

    def create_digital_display(self, title, initial_value, color):
        frame = QFrame()
        frame.setStyleSheet(f"background-color: #181828; border: 2px solid {color}; border-radius: 10px;")
        layout = QVBoxLayout(frame)
        
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(f"color: {color}; font-size: 16px; font-weight: bold; border: none;")
        title_lbl.setAlignment(Qt.AlignCenter)
        
        value_lbl = QLabel(initial_value)
        value_lbl.setStyleSheet(f"color: white; font-size: 48px; font-weight: bold; border: none;")
        value_lbl.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(title_lbl)
        layout.addWidget(value_lbl)
        
        # Guardamos la referencia al label del valor en el frame para poder actualizarlo luego
        frame.value_label = value_lbl 
        return frame

    def create_graph(self, title, color):
        plot = pg.PlotWidget(title=title)
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.setLabel('bottom', 'Tiempo', 's')
        plot.setBackground('#181828')
        
        # Estilizar ejes
        plot.getAxis('bottom').setPen(pg.mkPen('#aaa'))
        plot.getAxis('left').setPen(pg.mkPen('#aaa'))
        plot.getAxis('bottom').setTextPen(pg.mkPen('#aaa'))
        plot.getAxis('left').setTextPen(pg.mkPen('#aaa'))
        
        line = plot.plot(pen=pg.mkPen(color, width=3))
        
        # Relleno bajo la curva para efecto visual
        fill = pg.FillBetweenItem(curve1=line, curve2=pg.PlotDataItem([0,0], [0,0]), brush=pg.mkBrush(QColor(color).lighter(150).name() + '30'))
        plot.addItem(fill)
        
        return plot, line

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combo.currentText()
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.connect_btn.setText("Desconectar Sensor")
                self.connect_btn.setStyleSheet("background-color: #ff0055; color: white;")
                self.timer.start(20) # 50Hz refresh de GUI
            except Exception as e:
                print(f"Error al conectar: {e}")
        else:
            self.timer.stop()
            if self.serial_port:
                self.serial_port.close()
            self.is_connected = False
            self.connect_btn.setText("Conectar Sensor")
            self.connect_btn.setStyleSheet("")

    def tare_altitude(self):
        if len(self.alt_data) > 0:
            # Desactivar momentáneamente si ya estaba activa para obtener el valor real crudo
            current_raw_alt = self.alt_data[-1] 
            if self.use_relative_altitude:
                current_raw_alt += self.reference_altitude
                
            self.reference_altitude = current_raw_alt
            self.use_relative_altitude = True
            self.tare_btn.setText("📍 Altitud Cero Establecida")

    def read_serial_data(self):
        if not self.serial_port or not self.serial_port.in_waiting:
            return
            
        try:
            data = self.serial_port.read(self.serial_port.in_waiting).decode(errors='ignore')
            self.serial_buffer += data
            
            while '\n' in self.serial_buffer:
                line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                self.process_line(line.strip())
                
        except Exception as e:
            print(f"Error en lectura serial: {e}")

    def process_line(self, line):
        try:
            parts = [float(x) for x in line.split(',')]
            if len(parts) != 3:
                return
                
            temp, press, alt = parts
            
            # Aplicar Tara de Altitud si está activada
            if self.use_relative_altitude:
                alt = alt - self.reference_altitude
            
            # Actualizar colas de datos
            self.time_counter += 0.05 # Incremento aproximado basado en 20Hz
            self.time_data.append(self.time_counter)
            self.temp_data.append(temp)
            self.press_data.append(press)
            self.alt_data.append(alt)
            
            # Actualizar GUI
            self.update_gui(temp, press, alt)
            
        except ValueError:
            pass # Ignorar líneas corruptas al conectar

    def update_gui(self, temp, press, alt):
        # Actualizar displays numéricos
        self.lbl_temp.value_label.setText(f"{temp:.2f} °C")
        self.lbl_press.value_label.setText(f"{int(press)} Pa")
        
        alt_prefix = "+" if (self.use_relative_altitude and alt > 0) else ""
        self.lbl_alt.value_label.setText(f"{alt_prefix}{alt:.2f} m")
        
        # Actualizar datos de las líneas
        t = list(self.time_data)
        self.line_temp.setData(t, list(self.temp_data))
        self.line_press.setData(t, list(self.press_data))
        self.line_alt.setData(t, list(self.alt_data))

        # --- AUTO-SCROLL (Ventana Deslizante) ---
        if len(t) > 0:
            current_t = t[-1]
            # Como guardamos 500 puntos a 20Hz, son unos 25 segundos de historial
            window_size = 25.0 
            min_t = max(0, current_t - window_size)
            
            # Forzar a la gráfica a seguir el último punto en el eje X
            self.plot_temp.setXRange(min_t, current_t, padding=0.02)
            self.plot_press.setXRange(min_t, current_t, padding=0.02)
            self.plot_alt.setXRange(min_t, current_t, padding=0.02)

    def apply_dark_theme(self):
        dark_palette = QPalette()
        dark_palette.setColor(QPalette.Window, QColor(24, 24, 40))
        dark_palette.setColor(QPalette.WindowText, Qt.white)
        dark_palette.setColor(QPalette.Base, QColor(18, 18, 40))
        dark_palette.setColor(QPalette.Text, Qt.white)
        dark_palette.setColor(QPalette.Button, QColor(45, 45, 60))
        dark_palette.setColor(QPalette.ButtonText, Qt.white)
        QApplication.instance().setPalette(dark_palette)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BMP180Dashboard()
    window.show()
    sys.exit(app.exec_())