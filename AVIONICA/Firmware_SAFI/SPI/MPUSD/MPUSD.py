"""
IMU MPU6050 3D Simulator - File Playback Version
Professional GUI with Large Graphs and Real-time 3D Visualization from SD Card Data
"""

import sys
import os
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QFileDialog, 
                             QGridLayout, QGroupBox, QTabWidget, QSplitter)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from OpenGL.GL import *
from OpenGL.GLU import *
import math

# ==================== KALMAN FILTER ====================
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimation_error):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.estimate = 0.0
        
    def update(self, measurement):
        self.estimation_error += self.process_variance
        kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.estimation_error *= (1 - kalman_gain)
        return self.estimate

# ==================== IMU DATA PROCESSOR ====================
class IMUProcessor:
    def __init__(self):
        # Kalman filters for raw data
        self.kf_ax = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_ay = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_az = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_gx = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_gy = KalmanFilter(0.001, 0.1, 1.0)
        self.kf_gz = KalmanFilter(0.001, 0.1, 1.0)
        
        # Enhanced complementary filter parameters
        self.alpha = 0.98  # Gyro weight (98% gyro, 2% accel for smoother motion)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.dt = 0.01  # 100Hz sample rate
        
        # Calibration offsets 
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        self.yaw_offset = 0.0
        
        # Bias compensation
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.bias_alpha = 0.001  # Slow adaptation
        
        # For initialization and calibration
        self.initialized = False
        self.calibrated = False
        self.init_samples = 0
        self.calibration_samples = 0
        self.init_ax_sum = 0.0
        self.init_ay_sum = 0.0
        self.init_az_sum = 0.0
        self.calib_pitch_sum = 0.0
        self.calib_roll_sum = 0.0
        self.calib_yaw_sum = 0.0
        
    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180] range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def reset_calibration(self):
        """Reset calibration to recalibrate from current position"""
        self.calibrated = False
        self.calibration_samples = 0
        self.calib_pitch_sum = 0.0
        self.calib_roll_sum = 0.0
        self.calib_yaw_sum = 0.0
        print("🔄 Calibración reiniciada - calculando desde la posición actual...")
    
    def process(self, ax, ay, az, gx, gy, gz):
        # Apply Kalman filter to reduce noise
        ax_f = self.kf_ax.update(ax)
        ay_f = self.kf_ay.update(ay)
        az_f = self.kf_az.update(az)
        gx_f = self.kf_gx.update(gx)
        gy_f = self.kf_gy.update(gy)
        gz_f = self.kf_gz.update(gz)

        # Initialize with first samples (initial orientation estimation)
        if not self.initialized:
            self.init_samples += 1
            self.init_ax_sum += ax_f
            self.init_ay_sum += ay_f
            self.init_az_sum += az_f

            if self.init_samples >= 50:  # Use first 50 samples for initialization
                # Calculate initial angles from accelerometer
                ax_avg = self.init_ax_sum / self.init_samples
                ay_avg = self.init_ay_sum / self.init_samples
                az_avg = self.init_az_sum / self.init_samples

                # Initial pitch and roll from gravity vector
                self.pitch_offset = math.atan2(ay_avg, math.sqrt(ax_avg**2 + az_avg**2)) * 180 / math.pi
                self.roll_offset = math.atan2(-ax_avg, az_avg) * 180 / math.pi
                self.yaw_offset = 0.0

                self.pitch = 0.0
                self.roll = 0.0
                self.yaw = 0.0

                self.initialized = True
                self.movement_detected = False
                print(f"✓ IMU Inicializado - Offsets: Pitch={self.pitch_offset:.2f}°, Roll={self.roll_offset:.2f}°")
                return ax_f, ay_f, az_f, gx_f, gy_f, gz_f, 0.0, 0.0, 0.0

        # Calibration phase - just wait for a few more samples for stability
        if self.initialized and not self.calibrated:
            self.calibration_samples += 1
            acc_magnitude = math.sqrt(ax_f**2 + ay_f**2 + az_f**2)
            if abs(acc_magnitude - 1.0) > 0.15 or abs(gx_f) > 2 or abs(gy_f) > 2 or abs(gz_f) > 2:
                self.movement_detected = True
            else:
                self.movement_detected = False
                
            if self.calibration_samples >= 50 and not self.movement_detected:
                self.calibrated = True
                print(f"✓✓ CALIBRACIÓN COMPLETA ✓✓")
            return ax_f, ay_f, az_f, gx_f, gy_f, gz_f, 0.0, 0.0, 0.0

        # Apply gyro bias compensation (adaptive)
        gx_corrected = gx_f - self.gyro_bias_x
        gy_corrected = gy_f - self.gyro_bias_y
        gz_corrected = gz_f - self.gyro_bias_z

        # === IMPROVED ANGLE CALCULATION FROM ACCELEROMETER ===
        acc_magnitude = math.sqrt(ax_f**2 + ay_f**2 + az_f**2)

        # Only use accelerometer if magnitude is close to 1g
        accel_trust_factor = 1.0
        if abs(acc_magnitude - 1.0) > 0.3:
            accel_trust_factor = 0.1

        if acc_magnitude > 0.01:
            ax_n = ax_f / acc_magnitude
            ay_n = ay_f / acc_magnitude
            az_n = az_f / acc_magnitude

            # Calculate pitch and roll from normalized accelerometer
            pitch_acc = math.atan2(ay_n, math.sqrt(ax_n**2 + az_n**2)) * 180 / math.pi - self.pitch_offset
            roll_acc = math.atan2(-ax_n, az_n) * 180 / math.pi - self.roll_offset

            # Handle roll angle discontinuity at ±180°
            if az_n < 0:
                if roll_acc > 0:
                    roll_acc = 180 - roll_acc
                else:
                    roll_acc = -180 - roll_acc
        else:
            pitch_acc = self.pitch
            roll_acc = self.roll

        # === GYROSCOPE INTEGRATION ===
        pitch_rate = gy_corrected
        roll_rate = gx_corrected
        yaw_rate = gz_corrected

        # Apply rotation with smoothing
        pitch_gyro = self.pitch + pitch_rate * self.dt
        roll_gyro = self.roll + roll_rate * self.dt
        yaw_gyro = self.yaw + yaw_rate * self.dt

        # === COMPLEMENTARY FILTER WITH ADAPTIVE ALPHA ===
        adaptive_alpha = self.alpha + (1 - self.alpha) * (1 - accel_trust_factor)

        # Fuse gyroscope and accelerometer data
        self.pitch = adaptive_alpha * pitch_gyro + (1 - adaptive_alpha) * pitch_acc
        self.roll = adaptive_alpha * roll_gyro + (1 - adaptive_alpha) * roll_acc
        self.yaw = yaw_gyro  # Yaw from gyro only

        # Normalize angles to [-180, 180]
        self.pitch = self.normalize_angle(self.pitch)
        self.roll = self.normalize_angle(self.roll)
        self.yaw = self.normalize_angle(self.yaw)

        # === ADAPTIVE GYRO BIAS CORRECTION ===
        if abs(gx_corrected) < 2 and abs(gy_corrected) < 2 and abs(gz_corrected) < 2:
            if abs(acc_magnitude - 1.0) < 0.1:
                self.gyro_bias_x += self.bias_alpha * gx_corrected
                self.gyro_bias_y += self.bias_alpha * gy_corrected
                self.gyro_bias_z += self.bias_alpha * gz_corrected

        # Cálculo explícito de los ángulos de Euler
        pitch_exp, roll_exp, yaw_exp = self.calculate_euler_angles(ax_f, ay_f, az_f, gx_f, gy_f, gz_f)

        return ax_f, ay_f, az_f, gx_f, gy_f, gz_f, pitch_exp, roll_exp, yaw_exp

    def calculate_euler_angles(self, ax, ay, az, gx, gy, gz):
        roll = math.atan2(ay, az) * 180 / math.pi - self.roll_offset
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi - self.pitch_offset
        self.yaw += gz * self.dt
        self.yaw = self.normalize_angle(self.yaw)
        return pitch, roll, self.yaw

# ==================== ENHANCED 3D VISUALIZATION ====================
class Enhanced3DWidget(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setBackgroundColor('#0a0a0a')
        self.setCameraPosition(distance=8, elevation=20, azimuth=45)
        
        self.target_pitch = 0.0
        self.target_roll = 0.0
        self.target_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw = 0.0
        self.smoothing_factor = 0.3 
        
        self.create_enhanced_axes()
        self.create_imu_cube()
        
    def create_enhanced_axes(self):
        axis_length = 3.5
        axis_width = 4
        
        x_points = np.array([[0,0,0], [axis_length,0,0]])
        x_axis = gl.GLLinePlotItem(pos=x_points, color=(1,0,0,1), width=axis_width, antialias=True)
        self.addItem(x_axis)
        x_arrow = self.create_arrow([axis_length,0,0], [1,0,0], (1,0,0,1))
        self.addItem(x_arrow)
        
        y_points = np.array([[0,0,0], [0,axis_length,0]])
        y_axis = gl.GLLinePlotItem(pos=y_points, color=(0,1,0,1), width=axis_width, antialias=True)
        self.addItem(y_axis)
        y_arrow = self.create_arrow([0,axis_length,0], [0,1,0], (0,1,0,1))
        self.addItem(y_arrow)
        
        z_points = np.array([[0,0,0], [0,0,axis_length]])
        z_axis = gl.GLLinePlotItem(pos=z_points, color=(0,0,1,1), width=axis_width, antialias=True)
        self.addItem(z_axis)
        z_arrow = self.create_arrow([0,0,axis_length], [0,0,1], (0,0,1,1))
        self.addItem(z_arrow)
        
        grid = gl.GLGridItem()
        grid.setSize(10, 10)
        grid.setSpacing(1, 1)
        grid.translate(0, 0, -2)
        self.addItem(grid)
        
    def create_arrow(self, position, direction, color):
        arrow_size = 0.3
        pos = np.array(position)
        dir_vec = np.array(direction)
        
        if abs(dir_vec[2]) < 0.9:
            perp1 = np.cross(dir_vec, [0,0,1])
        else:
            perp1 = np.cross(dir_vec, [1,0,0])
        perp1 = perp1 / np.linalg.norm(perp1)
        perp2 = np.cross(dir_vec, perp1)
        
        base = pos - dir_vec * arrow_size
        points = np.array([
            pos, base + perp1 * arrow_size * 0.5, pos, base - perp1 * arrow_size * 0.5,
            pos, base + perp2 * arrow_size * 0.5, pos, base - perp2 * arrow_size * 0.5
        ])
        
        return gl.GLLinePlotItem(pos=points, color=color, width=3, mode='lines')
        
    def create_imu_cube(self):
        # FUSELAGE
        fuselage_vertices = np.array([
            [0, 1.5, 0], [-0.15, 1.2, 0], [0.15, 1.2, 0], [-0.15, 1.2, 0.1], [0.15, 1.2, 0.1],
            [-0.2, 0.5, 0], [0.2, 0.5, 0], [-0.2, 0.5, 0.15], [0.2, 0.5, 0.15],
            [-0.15, -0.8, 0], [0.15, -0.8, 0], [-0.15, -0.8, 0.1], [0.15, -0.8, 0.1],
            [0, -1.2, 0.05],
        ])
        fuselage_faces = np.array([
            [0, 1, 3], [0, 3, 4], [0, 4, 2], [0, 2, 1], [1, 2, 6], [1, 6, 5], [2, 4, 8], [2, 8, 6],
            [3, 7, 8], [3, 8, 4], [1, 5, 7], [1, 7, 3], [5, 6, 10], [5, 10, 9], [6, 8, 12], [6, 12, 10],
            [7, 11, 12], [7, 12, 8], [5, 9, 11], [5, 11, 7], [9, 10, 13], [10, 12, 13], [12, 11, 13], [11, 9, 13],
        ])
        
        # WINGS
        wing_vertices = np.array([
            [-0.2, 0.5, 0.1], [-1.5, 0.2, 0.1], [-0.2, 0.3, 0.1], [-1.3, 0.0, 0.1],
            [-0.2, 0.5, 0.15], [-1.5, 0.2, 0.12], [-0.2, 0.3, 0.15], [-1.3, 0.0, 0.12],
            [0.2, 0.5, 0.1], [1.5, 0.2, 0.1], [0.2, 0.3, 0.1], [1.3, 0.0, 0.1],
            [0.2, 0.5, 0.15], [1.5, 0.2, 0.12], [0.2, 0.3, 0.15], [1.3, 0.0, 0.12],
        ])
        wing_faces = np.array([
            [0, 1, 3], [0, 3, 2], [4, 5, 7], [4, 7, 6], [0, 4, 5], [0, 5, 1], [2, 6, 7], [2, 7, 3],
            [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3], [8, 9, 11], [8, 11, 10], [12, 13, 15], [12, 15, 14],
            [8, 12, 13], [8, 13, 9], [10, 14, 15], [10, 15, 11], [8, 10, 14], [8, 14, 12], [9, 13, 15], [9, 15, 11],
        ])
        
        # TAIL
        tail_vertices = np.array([
            [-0.05, -0.8, 0.1], [0.05, -0.8, 0.1], [-0.05, -0.6, 0.1], [0.05, -0.6, 0.1], [0, -1.0, 0.5],
        ])
        tail_faces = np.array([
            [0, 1, 4], [1, 3, 4], [3, 2, 4], [2, 0, 4],
        ])
        
        # HORIZONTAL STABILIZER
        h_stab_vertices = np.array([
            [-0.6, -0.9, 0.05], [-0.6, -1.1, 0.05], [0.6, -0.9, 0.05], [0.6, -1.1, 0.05],
            [-0.05, -0.9, 0.05], [0.05, -0.9, 0.05], [-0.05, -1.1, 0.05], [0.05, -1.1, 0.05],
        ])
        h_stab_faces = np.array([
            [0, 1, 6], [0, 6, 4], [2, 3, 7], [2, 7, 5],
        ])
        
        fuselage_colors = np.ones((len(fuselage_vertices), 4))
        fuselage_colors[:, :3] = [0.2, 0.5, 0.9]
        fuselage_colors[:, 3] = 0.9
        
        wing_colors = np.ones((len(wing_vertices), 4))
        wing_colors[:, :3] = [0.9, 0.2, 0.2]
        wing_colors[:, 3] = 0.9
        
        tail_colors = np.ones((len(tail_vertices), 4))
        tail_colors[:, :3] = [0.9, 0.7, 0.1]
        tail_colors[:, 3] = 0.9
        
        h_stab_colors = np.ones((len(h_stab_vertices), 4))
        h_stab_colors[:, :3] = [0.9, 0.7, 0.1]
        h_stab_colors[:, 3] = 0.9
        
        self.fuselage = gl.GLMeshItem(vertexes=fuselage_vertices, faces=fuselage_faces, vertexColors=fuselage_colors, smooth=True, drawEdges=True, edgeColor=(0.1, 0.1, 0.1, 1))
        self.wings = gl.GLMeshItem(vertexes=wing_vertices, faces=wing_faces, vertexColors=wing_colors, smooth=True, drawEdges=True, edgeColor=(0.1, 0.1, 0.1, 1))
        self.tail = gl.GLMeshItem(vertexes=tail_vertices, faces=tail_faces, vertexColors=tail_colors, smooth=True, drawEdges=True, edgeColor=(0.1, 0.1, 0.1, 1))
        self.h_stabilizer = gl.GLMeshItem(vertexes=h_stab_vertices, faces=h_stab_faces, vertexColors=h_stab_colors, smooth=True, drawEdges=True, edgeColor=(0.1, 0.1, 0.1, 1))
        
        self.addItem(self.fuselage)
        self.addItem(self.wings)
        self.addItem(self.tail)
        self.addItem(self.h_stabilizer)
        
        self.plane_x_axis = gl.GLLinePlotItem(pos=np.array([[-1.8, 0.3, 0.1], [1.8, 0.3, 0.1]]), color=(1, 0, 0, 1), width=4)
        self.plane_y_axis = gl.GLLinePlotItem(pos=np.array([[0, -1.5, 0.1], [0, 1.8, 0.1]]), color=(0, 1, 0, 1), width=4)
        self.plane_z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0.3, -0.3], [0, 0.3, 0.8]]), color=(0, 0, 1, 1), width=4)
        
        self.addItem(self.plane_x_axis)
        self.addItem(self.plane_y_axis)
        self.addItem(self.plane_z_axis)
        
    def update_orientation(self, pitch, roll, yaw):
        self.target_pitch = pitch
        self.target_roll = roll
        self.target_yaw = yaw
        
        self.current_pitch += (self.target_pitch - self.current_pitch) * self.smoothing_factor
        self.current_roll += (self.target_roll - self.current_roll) * self.smoothing_factor
        self.current_yaw += (self.target_yaw - self.current_yaw) * self.smoothing_factor
        
        for part in [self.fuselage, self.wings, self.tail, self.h_stabilizer]:
            part.resetTransform()
            part.rotate(self.current_roll, 1, 0, 0)
            part.rotate(self.current_pitch, 0, 1, 0)
            part.rotate(self.current_yaw, 0, 0, 1)
        
        for ax_line in [self.plane_x_axis, self.plane_y_axis, self.plane_z_axis]:
            ax_line.resetTransform()
            ax_line.rotate(self.current_roll, 1, 0, 0)
            ax_line.rotate(self.current_pitch, 0, 1, 0)
            ax_line.rotate(self.current_yaw, 0, 0, 1)


# ==================== MAIN APPLICATION ====================
class IMUSimulator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU MPU6050 3D Simulator - Lector de Archivos SD")
        self.setGeometry(50, 50, 1800, 1000)
        
        # Data storage
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.ax_data = deque(maxlen=self.max_points)
        self.ay_data = deque(maxlen=self.max_points)
        self.az_data = deque(maxlen=self.max_points)
        self.gx_data = deque(maxlen=self.max_points)
        self.gy_data = deque(maxlen=self.max_points)
        self.gz_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.roll_data = deque(maxlen=self.max_points)
        self.yaw_data = deque(maxlen=self.max_points)
        self.time_counter = 0
        
        # File reading variables
        self.file_path = None
        self.file_obj = None
        self.is_playing = False
        
        # IMU Processor
        self.imu_processor = IMUProcessor()
        
        # Setup UI
        self.setup_ui()
        self.apply_dark_theme()
        
        # Timer for file reading (Simulate 100Hz = 10ms per line)
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_file_data)
        
    def create_gauge(self, title, color):
        gauge = pg.PlotWidget()
        gauge.setFixedHeight(260)
        gauge.setFixedWidth(260)
        gauge.setBackground('#181828')
        gauge.hideAxis('bottom')
        gauge.hideAxis('left')
        gauge.setAspectLocked(True)
        theta = np.linspace(0, 2 * np.pi, 200)
        x = 120 * np.cos(theta)
        y = 120 * np.sin(theta)
        circle = pg.PlotDataItem(x, y, pen=pg.mkPen('#444', width=6))
        gauge.addItem(circle)
        needle = pg.PlotDataItem()
        gauge.addItem(needle)
        label = pg.TextItem(title, color=color, anchor=(0.5, 0.5))
        label.setPos(0, 150)
        gauge.addItem(label)
        value_label = pg.TextItem("0.0°", color=color, anchor=(0.5, 0.5))
        value_label.setPos(0, -150)
        gauge.addItem(value_label)
        return gauge, needle, value_label

    def create_euler_gauges_tab(self):
        widget = QWidget()
        layout = QHBoxLayout(widget)
        self.euler_gauges = {}
        for name, color in zip(["Pitch", "Roll", "Yaw"], ["#00eaff", "#ffb300", "#ff0055"]):
            gauge, needle, value_label = self.create_gauge(name, color)
            self.euler_gauges[name.lower()] = (needle, value_label)
            layout.addWidget(gauge)
        widget.setLayout(layout)
        return widget

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # ========== TOP CONTROL PANEL ==========
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)
        
        # ========== MAIN CONTENT SPLITTER ==========
        splitter = QSplitter(Qt.Horizontal)
        
        # ========== LEFT SIDE - 3D VISUALIZATION ==========
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(10)
        
        title_3d = QLabel("🎯 Visualización 3D en Tiempo Real de Archivo SD")
        title_3d.setFont(QFont("Arial", 13, QFont.Bold))
        title_3d.setAlignment(Qt.AlignCenter)
        title_3d.setStyleSheet("background: #23233a; padding: 8px; border-radius: 8px; color: #00eaff; border: 2px solid #00eaff;")
        left_layout.addWidget(title_3d)
        
        self.status_label = QLabel("Esperando archivo...")
        self.status_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #aaa; background: #23233a; padding: 8px; border-radius: 6px;")
        left_layout.addWidget(self.status_label)
        
        self.view_3d = Enhanced3DWidget()
        left_layout.addWidget(self.view_3d, stretch=1)
        left_widget.setLayout(left_layout)
        splitter.addWidget(left_widget)

        # ========== RIGHT SIDE - TABS FOR GRAPHS ========== 
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(10)
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet('''
            QTabBar::tab { background: #23233a; color: #fff; font-weight: bold; padding: 8px 32px; min-width: 180px; min-height: 32px; border: 1px solid #23233a; border-bottom: none; }
            QTabBar::tab:selected { background: #181828; color: #00eaff; border: 2px solid #00eaff; border-bottom: none; }
            QTabWidget::pane { border: 2px solid #23233a; top: -1px; background: #181828; }
        ''')
        self.tabs.addTab(self.create_sensor_tab('Acelerómetro', ['ax', 'ay', 'az']), "Acelerómetro")
        self.tabs.addTab(self.create_sensor_tab('Giroscopio', ['gx', 'gy', 'gz']), "Giroscopio")
        self.tabs.addTab(self.create_euler_gauges_tab(), "Ángulos Euler")
        right_layout.addWidget(self.tabs)
        right_widget.setLayout(right_layout)
        splitter.addWidget(right_widget)
        splitter.setSizes([900, 900])
        main_layout.addWidget(splitter, stretch=1)

    def create_sensor_tab(self, title, keys):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        plot = pg.PlotWidget(title=title)
        plot.addLegend(labelTextColor='w', brush=pg.mkBrush(30,30,40,200))
        plot.showGrid(x=True, y=True)
        plot.setLabel('bottom', 'Tiempo', 's', color='w')
        plot.setLabel('left', title, color='w')
        plot.getAxis('bottom').setPen(pg.mkPen('w'))
        plot.getAxis('left').setPen(pg.mkPen('w'))
        plot.getAxis('bottom').setTextPen(pg.mkPen('w'))
        plot.getAxis('left').setTextPen(pg.mkPen('w'))
        plot.setBackground('#181828')
        colors = ['#00eaff', '#ffb300', '#ff0055']
        self.graph_lines = getattr(self, 'graph_lines', {})
        for i, key in enumerate(keys):
            line = plot.plot(pen=pg.mkPen(colors[i], width=2), name=key.upper())
            self.graph_lines[key] = line
        layout.addWidget(plot)
        value_layout = QHBoxLayout()
        for i, key in enumerate(keys):
            label = QLabel(f"{key.upper()}: 0.00")
            label.setObjectName(f"label_{key}")
            label.setStyleSheet(f"color: {colors[i]}; font-size: 18px; font-weight: bold; padding: 8px;")
            value_layout.addWidget(label)
        layout.addLayout(value_layout)
        widget.setLayout(layout)
        return widget

    def create_control_panel(self):
        panel = QGroupBox("Control de Archivo SD")
        panel.setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; color: #aaa; border: 1.5px solid #aaa; border-radius: 6px; margin-top: 8px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px 0 4px; }")
        layout = QHBoxLayout(panel)
        
        self.file_label = QLabel("Ningún archivo seleccionado")
        self.file_label.setStyleSheet("color: #fff; padding: 4px; border: 1px solid #444; background: #222; border-radius: 4px;")
        
        self.select_btn = QPushButton("📁 Seleccionar Archivo")
        self.select_btn.setMinimumHeight(32)
        self.select_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 4px 12px;")
        self.select_btn.clicked.connect(self.select_file)
        
        self.play_btn = QPushButton("▶ Reproducir")
        self.play_btn.setMinimumHeight(32)
        self.play_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 4px 12px;")
        self.play_btn.clicked.connect(self.toggle_playback)
        self.play_btn.setEnabled(False)
        
        self.calibrate_btn = QPushButton("Recalibrar")
        self.calibrate_btn.setMinimumHeight(32)
        self.calibrate_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 4px 12px;")
        self.calibrate_btn.clicked.connect(self.imu_processor.reset_calibration)
        
        layout.addWidget(QLabel("<b>Archivo:</b>"))
        layout.addWidget(self.file_label, stretch=1)
        layout.addWidget(self.select_btn)
        layout.addWidget(self.play_btn)
        layout.addWidget(self.calibrate_btn)
        return panel

    def select_file(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Seleccionar archivo de datos IMU", "", "Archivos de texto/CSV (*.txt *.csv);;Todos los archivos (*)", options=options)
        if file_name:
            self.file_path = file_name
            self.file_label.setText(os.path.basename(file_name))
            self.play_btn.setEnabled(True)
            self.status_label.setText("Archivo listo para reproducir.")
            self.status_label.setStyleSheet("color: #00eaff; background: #23233a; padding: 8px; border-radius: 6px;")

    def toggle_playback(self):
        if not self.is_playing:
            # Empezar a reproducir
            try:
                if self.file_obj:
                    self.file_obj.close()
                
                self.file_obj = open(self.file_path, 'r')
                self.is_playing = True
                self.play_btn.setText("⏸ Pausar/Detener")
                self.timer.start(10) # 10ms = 100Hz simulados
                self.status_label.setText("▶ Leyendo archivo...")
            except Exception as e:
                print(f"Error al abrir el archivo: {e}")
                self.status_label.setText(f"Error: {e}")
        else:
            # Pausar/Detener
            self.is_playing = False
            self.timer.stop()
            self.play_btn.setText("▶ Reproducir")
            self.status_label.setText("⏸ Pausado")

    def read_file_data(self):
        if not self.is_playing or not self.file_obj:
            return
            
        try:
            line = self.file_obj.readline()
            
            # Chequear Fin del Archivo (EOF)
            if not line:
                self.toggle_playback()
                self.status_label.setText("✓ Fin del archivo alcanzado.")
                self.status_label.setStyleSheet("color: #ffb300; background: #23233a; padding: 8px; border-radius: 6px;")
                return
                
            self.process_line(line.strip())
            
        except Exception as e:
            print(f"Error de lectura del archivo: {e}")
            self.toggle_playback()

    def process_line(self, line):
        # Ignorar líneas vacías o encabezados que a veces se guardan en el SD
        if not line or line.startswith(('#', 'a', 'x', 't')): 
            return
            
        try:
            parts = [float(x) for x in line.split(',')]
            if len(parts) != 6:
                return
                
            ax, ay, az, gx, gy, gz = parts
            ax, ay, az, gx, gy, gz, pitch, roll, yaw = self.imu_processor.process(ax, ay, az, gx, gy, gz)
            
            self.time_counter += 0.01
            self.time_data.append(self.time_counter)
            self.ax_data.append(ax)
            self.ay_data.append(ay)
            self.az_data.append(az)
            self.gx_data.append(gx)
            self.gy_data.append(gy)
            self.gz_data.append(gz)
            self.pitch_data.append(pitch)
            self.roll_data.append(roll)
            self.yaw_data.append(yaw)
            
            self.update_graphs()
            self.view_3d.update_orientation(pitch, roll, yaw)
            
            if hasattr(self, 'euler_gauges'):
                self.update_euler_gauges(pitch, roll, yaw)
                
            # Estado de calibración
            if not self.imu_processor.initialized:
                self.status_label.setText("⏳ Inicializando...")
            elif not self.imu_processor.calibrated:
                if getattr(self.imu_processor, 'movement_detected', False):
                    self.status_label.setText("⚠️ Movimiento en lectura. Ignorando calibración temporalmente.")
                    self.status_label.setStyleSheet("color: #ff0055; background: #23233a; padding: 8px; border-radius: 6px;")
                else:
                    self.status_label.setText("⏳ Calibrando posición inicial de los datos...")
            
        except ValueError:
            # Ignorar líneas que no se pueden convertir a float (posible basura en la SD)
            pass
        except Exception as e:
            print(f"Error en process_line: {e}")

    def update_graphs(self):
        if not hasattr(self, 'graph_lines'):
            return
        t = list(self.time_data)
        for key, data in zip(['ax','ay','az','gx','gy','gz','pitch','roll','yaw'],
                             [self.ax_data,self.ay_data,self.az_data,self.gx_data,self.gy_data,self.gz_data,self.pitch_data,self.roll_data,self.yaw_data]):
            if key in self.graph_lines:
                self.graph_lines[key].setData(t, list(data))
                label = self.tabs.findChild(QLabel, f"label_{key}")
                if label:
                    label.setText(f"{key.upper()}: {data[-1]:.2f}")

    def update_euler_gauges(self, pitch, roll, yaw):
        for angle, value in zip(["pitch", "roll", "yaw"], [pitch, roll, yaw]):
            needle, value_label = self.euler_gauges[angle]
            theta = (value - 90) * math.pi / 180
            x = 105 * math.cos(theta)
            y = 105 * math.sin(theta)
            needle.setData([0, x], [0, y], pen=pg.mkPen(width=10, color=value_label.color))
            value_label.setText(f"{value:.1f}°")

    def apply_dark_theme(self):
        dark_palette = QPalette()
        dark_palette.setColor(QPalette.Window, QColor(24, 24, 40))
        dark_palette.setColor(QPalette.WindowText, Qt.white)
        dark_palette.setColor(QPalette.Base, QColor(18, 18, 40))
        dark_palette.setColor(QPalette.AlternateBase, QColor(30, 30, 40))
        dark_palette.setColor(QPalette.ToolTipBase, Qt.white)
        dark_palette.setColor(QPalette.ToolTipText, Qt.white)
        dark_palette.setColor(QPalette.Text, Qt.white)
        dark_palette.setColor(QPalette.Button, QColor(45, 45, 60))
        dark_palette.setColor(QPalette.ButtonText, Qt.white)
        dark_palette.setColor(QPalette.BrightText, Qt.red)
        dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
        dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        dark_palette.setColor(QPalette.HighlightedText, Qt.black)
        QApplication.instance().setPalette(dark_palette)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = IMUSimulator()
    window.show()
    sys.exit(app.exec_())