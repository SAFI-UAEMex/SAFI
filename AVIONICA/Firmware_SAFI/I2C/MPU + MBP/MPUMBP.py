"""
Aerospace Mission Control Dashboard
Integrates MPU6050 (3D IMU) + BMP180 (Atmospherics)
"""

import sys
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QComboBox, 
                             QGroupBox, QTabWidget, QSplitter, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from OpenGL.GL import *
import serial
import serial.tools.list_ports
import math

# ==================== FILTROS Y PROCESAMIENTO (De tu código original) ====================
class KalmanFilter:
    def __init__(self, pv, mv, ee):
        self.pv, self.mv, self.ee, self.est = pv, mv, ee, 0.0
    def update(self, mea):
        self.ee += self.pv
        kg = self.ee / (self.ee + self.mv)
        self.est += kg * (mea - self.est)
        self.ee *= (1 - kg)
        return self.est

class IMUProcessor:
    def __init__(self):
        self.kf_ax = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_ay = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_az = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_gx = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_gy = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_gz = KalmanFilter(0.001, 0.1, 1.0)
        self.alpha, self.pitch, self.roll, self.yaw, self.dt = 0.98, 0.0, 0.0, 0.0, 0.02
        self.pitch_offset, self.roll_offset = 0.0, 0.0
        self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z = 0.0, 0.0, 0.0
        self.initialized, self.calibrated, self.init_samples = False, False, 0
        self.init_ax_sum, self.init_ay_sum, self.init_az_sum = 0.0, 0.0, 0.0

    def reset_calibration(self):
        self.calibrated, self.initialized, self.init_samples = False, False, 0

    def process(self, ax, ay, az, gx, gy, gz):
        ax_f, ay_f, az_f = self.kf_ax.update(ax), self.kf_ay.update(ay), self.kf_az.update(az)
        gx_f, gy_f, gz_f = self.kf_gx.update(gx), self.kf_gy.update(gy), self.kf_gz.update(gz)

        if not self.initialized:
            self.init_samples += 1
            self.init_ax_sum += ax_f; self.init_ay_sum += ay_f; self.init_az_sum += az_f
            if self.init_samples >= 50:
                a_mag = math.sqrt((self.init_ax_sum/50)**2 + (self.init_az_sum/50)**2)
                self.pitch_offset = math.atan2(self.init_ay_sum/50, a_mag) * 180/math.pi
                self.roll_offset = math.atan2(-self.init_ax_sum/50, self.init_az_sum/50) * 180/math.pi
                self.pitch, self.roll, self.yaw = 0.0, 0.0, 0.0
                self.initialized = True
            return ax_f, ay_f, az_f, gx_f, gy_f, gz_f, 0.0, 0.0, 0.0

        gx_c, gy_c, gz_c = gx_f - self.gyro_bias_x, gy_f - self.gyro_bias_y, gz_f - self.gyro_bias_z
        acc_mag = math.sqrt(ax_f**2 + ay_f**2 + az_f**2)
        trust = 1.0 if abs(acc_mag - 1.0) <= 0.3 else 0.1

        pitch_acc, roll_acc = self.pitch, self.roll
        if acc_mag > 0.01:
            ax_n, ay_n, az_n = ax_f/acc_mag, ay_f/acc_mag, az_f/acc_mag
            pitch_acc = math.atan2(ay_n, math.sqrt(ax_n**2 + az_n**2)) * 180/math.pi - self.pitch_offset
            roll_acc = math.atan2(-ax_n, az_n) * 180/math.pi - self.roll_offset
            if az_n < 0: roll_acc = (180 - roll_acc) if roll_acc > 0 else (-180 - roll_acc)

        adapt_alpha = self.alpha + (1 - self.alpha) * (1 - trust)
        self.pitch = adapt_alpha * (self.pitch + gy_c * self.dt) + (1 - adapt_alpha) * pitch_acc
        self.roll = adapt_alpha * (self.roll + gx_c * self.dt) + (1 - adapt_alpha) * roll_acc
        self.yaw += gz_c * self.dt

        for a in ['pitch', 'roll', 'yaw']:
            val = getattr(self, a)
            while val > 180: val -= 360
            while val < -180: val += 360
            setattr(self, a, val)

        return ax_f, ay_f, az_f, gx_f, gy_f, gz_f, self.pitch, self.roll, self.yaw

# ==================== WIDGET 3D ====================
class Enhanced3DWidget(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setBackgroundColor('#0a0a0a')
        self.setCameraPosition(distance=8, elevation=20, azimuth=45)
        self.curr_p, self.curr_r, self.curr_y = 0.0, 0.0, 0.0
        self.create_imu_cube()

    def create_imu_cube(self):
        # Geometría simplificada del avión para optimizar espacio en el script
        fuse_v = np.array([[0,1.5,0], [-0.15,1.2,0], [0.15,1.2,0], [-0.15,1.2,0.1], [0.15,1.2,0.1],
                           [-0.15,-0.8,0], [0.15,-0.8,0], [-0.15,-0.8,0.1], [0.15,-0.8,0.1]])
        fuse_f = np.array([[0,1,3], [0,3,4], [0,4,2], [0,2,1], [1,5,7], [1,7,3], [2,4,8], [2,8,6], [5,6,8], [5,8,7], [3,7,8], [3,8,4]])
        fuse_c = np.ones((len(fuse_v), 4)); fuse_c[:,:3] = [0.2,0.5,0.9]
        self.fuselage = gl.GLMeshItem(vertexes=fuse_v, faces=fuse_f, vertexColors=fuse_c, smooth=True, drawEdges=True, edgeColor=(0,0,0,1))
        
        wing_v = np.array([[-0.15,0.5,0.1], [-1.5,0.2,0.1], [-0.15,0.3,0.1], [-1.3,0.0,0.1],
                           [0.15,0.5,0.1], [1.5,0.2,0.1], [0.15,0.3,0.1], [1.3,0.0,0.1]])
        wing_f = np.array([[0,1,3],[0,3,2], [4,6,7],[4,7,5]])
        wing_c = np.ones((len(wing_v), 4)); wing_c[:,:3] = [0.9,0.2,0.2]
        self.wings = gl.GLMeshItem(vertexes=wing_v, faces=wing_f, vertexColors=wing_c, smooth=True, drawEdges=True, edgeColor=(0,0,0,1))

        self.addItem(self.fuselage)
        self.addItem(self.wings)
        self.addItem(gl.GLGridItem())

    def update_orientation(self, p, r, y):
        self.curr_p += (p - self.curr_p) * 0.3
        self.curr_r += (r - self.curr_r) * 0.3
        self.curr_y += (y - self.curr_y) * 0.3
        for part in [self.fuselage, self.wings]:
            part.resetTransform()
            part.rotate(self.curr_r, 1, 0, 0)
            part.rotate(self.curr_p, 0, 1, 0)
            part.rotate(self.curr_y, 0, 0, 1)

# ==================== MAIN DASHBOARD ====================
class AeroSpaceDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sistema Telemétrico Integrado: MPU6050 + BMP180")
        self.setGeometry(50, 50, 1600, 900)
        
        self.max_pts = 400
        self.time_data = deque(maxlen=self.max_pts)
        self.ax, self.ay, self.az = deque(maxlen=self.max_pts), deque(maxlen=self.max_pts), deque(maxlen=self.max_pts)
        self.gx, self.gy, self.gz = deque(maxlen=self.max_pts), deque(maxlen=self.max_pts), deque(maxlen=self.max_pts)
        self.pitch, self.roll, self.yaw = deque(maxlen=self.max_pts), deque(maxlen=self.max_pts), deque(maxlen=self.max_pts)
        self.temp, self.press, self.alt = deque(maxlen=self.max_pts), deque(maxlen=self.max_pts), deque(maxlen=self.max_pts)
        
        self.t_counter, self.ref_alt, self.use_rel_alt = 0, 0.0, False
        self.serial_port, self.is_conn, self.buffer = None, False, ""
        self.imu = IMUProcessor()
        
        self.setup_ui()
        self.apply_dark_theme()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)

    def setup_ui(self):
        main = QWidget()
        self.setCentralWidget(main)
        layout = QVBoxLayout(main)
        
        # Panel de Control
        ctrl = QGroupBox("Control Principal")
        c_lay = QHBoxLayout(ctrl)
        self.cmb_port = QComboBox()
        self.cmb_port.addItems([p.device for p in serial.tools.list_ports.comports()])
        
        btn_conn = QPushButton("Conectar Sistema"); btn_conn.clicked.connect(self.toggle_conn)
        btn_cal_imu = QPushButton("🔄 Recalibrar IMU"); btn_cal_imu.clicked.connect(self.imu.reset_calibration)
        self.btn_tare = QPushButton("📍 Cero Altitud (Tara)"); self.btn_tare.clicked.connect(self.tare_alt)
        self.btn_tare.setStyleSheet("background: #ffb300; color: #000; font-weight: bold;")
        
        c_lay.addWidget(QLabel("Puerto:")); c_lay.addWidget(self.cmb_port)
        c_lay.addWidget(btn_conn); c_lay.addWidget(btn_cal_imu); c_lay.addWidget(self.btn_tare); c_lay.addStretch()
        layout.addWidget(ctrl)
        
        # Splitter Central
        split = QSplitter(Qt.Horizontal)
        
        # Izquierda: 3D + Displays BMP
        left_w = QWidget(); l_lay = QVBoxLayout(left_w)
        self.view_3d = Enhanced3DWidget()
        l_lay.addWidget(self.view_3d, stretch=2)
        
        self.lbl_alt = self.create_lcd("ALTITUD", "0.00 m", "#ffb300")
        self.lbl_temp = self.create_lcd("TEMPERATURA", "0.00 °C", "#ff0055")
        self.lbl_press = self.create_lcd("PRESIÓN", "0 Pa", "#00eaff")
        l_lay.addWidget(self.lbl_alt); l_lay.addWidget(self.lbl_temp); l_lay.addWidget(self.lbl_press)
        
        # Derecha: Tabs Gráficas
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("QTabBar::tab { background: #23233a; padding: 10px 20px; font-weight: bold; } QTabBar::tab:selected { background: #181828; color: #00eaff; border-top: 2px solid #00eaff; }")
        
        self.tabs.addTab(self.create_multi_plot(['ax','ay','az'], "Acelerómetro (g)", ['#00eaff','#ffb300','#ff0055']), "Acelerómetro")
        self.tabs.addTab(self.create_multi_plot(['gx','gy','gz'], "Giroscopio (°/s)", ['#00eaff','#ffb300','#ff0055']), "Giroscopio")
        self.tabs.addTab(self.create_multi_plot(['pitch','roll','yaw'], "Euler (°)", ['#00eaff','#ffb300','#ff0055']), "Ángulos")
        self.tabs.addTab(self.create_bmp_plots(), "Atmosféricos (BMP180)")
        
        split.addWidget(left_w); split.addWidget(self.tabs)
        split.setSizes([500, 1100])
        layout.addWidget(split, stretch=1)

    def create_lcd(self, title, val, color):
        f = QFrame(); f.setStyleSheet(f"background: #181828; border: 1px solid {color}; border-radius: 5px;")
        lay = QVBoxLayout(f)
        t = QLabel(title); t.setStyleSheet(f"color: {color}; font-weight: bold; border: none; font-size: 14px;")
        v = QLabel(val); v.setStyleSheet("color: white; font-weight: bold; border: none; font-size: 32px;")
        v.setAlignment(Qt.AlignCenter); t.setAlignment(Qt.AlignCenter)
        lay.addWidget(t); lay.addWidget(v)
        f.val_lbl = v
        return f

    def create_multi_plot(self, keys, title, colors):
        w = QWidget(); lay = QVBoxLayout(w)
        p = pg.PlotWidget(title=title); p.addLegend()
        p.showGrid(x=True, y=True, alpha=0.3); p.setBackground('#181828')
        self.lines = getattr(self, 'lines', {})
        for i, k in enumerate(keys):
            self.lines[k] = p.plot(pen=pg.mkPen(colors[i], width=2), name=k.upper())
        lay.addWidget(p)
        w.plot_ref = p # Guardar ref para el scroll
        return w

    def create_bmp_plots(self):
        w = QWidget(); lay = QVBoxLayout(w)
        self.p_alt = pg.PlotWidget(title="Altitud (m)"); self.l_alt = self.p_alt.plot(pen=pg.mkPen('#ffb300', width=2))
        self.p_temp = pg.PlotWidget(title="Temperatura (°C)"); self.l_temp = self.p_temp.plot(pen=pg.mkPen('#ff0055', width=2))
        self.p_press = pg.PlotWidget(title="Presión (Pa)"); self.l_press = self.p_press.plot(pen=pg.mkPen('#00eaff', width=2))
        
        for p in [self.p_alt, self.p_temp, self.p_press]:
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setBackground('#181828')
            lay.addWidget(p)
        return w

    def toggle_conn(self):
        if not self.is_conn:
            try:
                self.serial_port = serial.Serial(self.cmb_port.currentText(), 115200, timeout=0.1)
                self.is_conn = True
                self.timer.start(20) # 50Hz
            except: pass
        else:
            self.timer.stop(); self.serial_port.close(); self.is_conn = False

    def tare_alt(self):
        if len(self.alt) > 0:
            c = self.alt[-1] + self.ref_alt if self.use_rel_alt else self.alt[-1]
            self.ref_alt, self.use_rel_alt = c, True
            self.btn_tare.setText("📍 Tara Activa")

    def read_serial(self):
        if not self.serial_port or not self.serial_port.in_waiting: return
        try:
            self.buffer += self.serial_port.read(self.serial_port.in_waiting).decode(errors='ignore')
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                self.process(line.strip())
        except: pass

    def process(self, line):
        try:
            pts = [float(x) for x in line.split(',')]
            if len(pts) != 9: return
            ax, ay, az, gx, gy, gz, t_raw, p_raw, a_raw = pts
            
            # Procesar IMU
            ax_f, ay_f, az_f, gx_f, gy_f, gz_f, p, r, y = self.imu.process(ax, ay, az, gx, gy, gz)
            
            # Procesar Altitud
            if self.use_rel_alt: a_raw -= self.ref_alt
            
            self.t_counter += 0.02 # 50Hz = 0.02s
            self.time_data.append(self.t_counter)
            
            for q, v in zip([self.ax, self.ay, self.az, self.gx, self.gy, self.gz, self.pitch, self.roll, self.yaw, self.temp, self.press, self.alt],
                            [ax_f, ay_f, az_f, gx_f, gy_f, gz_f, p, r, y, t_raw, p_raw, a_raw]):
                q.append(v)
            
            self.update_gui()
        except: pass

    def update_gui(self):
        if not self.time_data: return
        
        # 3D y Displays
        self.view_3d.update_orientation(self.pitch[-1], self.roll[-1], self.yaw[-1])
        pre = "+" if (self.use_rel_alt and self.alt[-1] > 0) else ""
        self.lbl_alt.val_lbl.setText(f"{pre}{self.alt[-1]:.2f} m")
        self.lbl_temp.val_lbl.setText(f"{self.temp[-1]:.2f} °C")
        self.lbl_press.val_lbl.setText(f"{int(self.press[-1])} Pa")
        
        # Gráficas
        t = list(self.time_data)
        for k, dq in zip(['ax','ay','az','gx','gy','gz','pitch','roll','yaw'], 
                         [self.ax, self.ay, self.az, self.gx, self.gy, self.gz, self.pitch, self.roll, self.yaw]):
            if k in self.lines: self.lines[k].setData(t, list(dq))
            
        self.l_alt.setData(t, list(self.alt))
        self.l_temp.setData(t, list(self.temp))
        self.l_press.setData(t, list(self.press))
        
        # Auto-Scroll (15 segundos)
        curr_t = t[-1]
        min_t = max(0, curr_t - 15.0)
        
        # Aplicar scroll a los de la pestaña actual para ahorrar CPU
        idx = self.tabs.currentIndex()
        if idx < 3:
            self.tabs.widget(idx).plot_ref.setXRange(min_t, curr_t, padding=0.02)
        elif idx == 3:
            self.p_alt.setXRange(min_t, curr_t, padding=0.02)
            self.p_temp.setXRange(min_t, curr_t, padding=0.02)
            self.p_press.setXRange(min_t, curr_t, padding=0.02)

    def apply_dark_theme(self):
        pal = QPalette()
        for r, c in [(QPalette.Window, QColor(24,24,40)), (QPalette.WindowText, Qt.white), 
                     (QPalette.Base, QColor(18,18,40)), (QPalette.Text, Qt.white), 
                     (QPalette.Button, QColor(45,45,60)), (QPalette.ButtonText, Qt.white)]:
            pal.setColor(r, c)
        QApplication.instance().setPalette(pal)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = AeroSpaceDashboard()
    win.show()
    sys.exit(app.exec_())