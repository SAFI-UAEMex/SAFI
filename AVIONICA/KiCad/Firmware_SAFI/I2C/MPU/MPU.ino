/*
 * Arduino + MPU6050 IMU Sensor
 * High-frequency data streaming for Python visualization
 * 
 * Conexiones (Arduino Uno / Nano):
 * MPU6050 VCC  -> Arduino 5V (o 3.3V dependiendo de tu módulo, el GY-521 soporta 5V)
 * MPU6050 GND  -> Arduino GND
 * MPU6050 SCL  -> Arduino A5
 * MPU6050 SDA  -> Arduino A4
 */

#include <Wire.h>

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// MPU6050 Registers
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

// Conversion factors
#define ACCEL_SCALE 16384.0  // Para ±2g
#define GYRO_SCALE  131.0    // Para ±250°/s

// Timing
unsigned long lastTime = 0;
const int SAMPLE_RATE = 100; // Hz
const int SAMPLE_PERIOD = 1000 / SAMPLE_RATE; // ms

// Raw sensor data
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t temperature_raw;

// Calibration offsets (adjust after calibration)
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

void setup() {
  Serial.begin(115200);
  
  // Inicialización I2C para Arduino (usa A4 y A5 por defecto)
  Wire.begin(); 
  Wire.setClock(400000); // 400kHz I2C
  
  delay(100);
  
  // Initialize MPU6050
  initMPU6050();
  
  // Calibration (optional - keep sensor still)
  Serial.println("Calibrating... Keep sensor still!");
  calibrateSensor();
  Serial.println("Calibration complete!");
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= SAMPLE_PERIOD) {
    lastTime = currentTime;
    
    // Read sensor data
    readMPU6050();
    
    // Convert to real units
    float ax = (ax_raw / ACCEL_SCALE) - ax_offset; // g
    float ay = (ay_raw / ACCEL_SCALE) - ay_offset;
    float az = (az_raw / ACCEL_SCALE) - az_offset;
    
    float gx = (gx_raw / GYRO_SCALE) - gx_offset; // °/s
    float gy = (gy_raw / GYRO_SCALE) - gy_offset;
    float gz = (gz_raw / GYRO_SCALE) - gz_offset;
    
    // Send data in CSV format: ax,ay,az,gx,gy,gz
    Serial.print(ax, 4);
    Serial.print(",");
    Serial.print(ay, 4);
    Serial.print(",");
    Serial.print(az, 4);
    Serial.print(",");
    Serial.print(gx, 4);
    Serial.print(",");
    Serial.print(gy, 4);
    Serial.print(",");
    Serial.println(gz, 4);
  }
}

void initMPU6050() {
  // Wake up MPU6050
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);
  
  // Configure DLPF (Digital Low Pass Filter)
  // DLPF_CFG = 3 -> Bandwidth ~44Hz
  writeRegister(CONFIG, 0x03);
  
  // Configure Gyroscope (±250 °/s)
  writeRegister(GYRO_CONFIG, 0x00);
  
  // Configure Accelerometer (±2g)
  writeRegister(ACCEL_CONFIG, 0x00);
  
  delay(100);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  // Read accelerometer (6 bytes)
  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();
  
  // Skip temperature (2 bytes)
  temperature_raw = (Wire.read() << 8) | Wire.read();
  
  // Read gyroscope (6 bytes)
  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();
}

void calibrateSensor() {
  const int samples = 1000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    
    ax_sum += ax_raw;
    ay_sum += ay_raw;
    az_sum += az_raw;
    gx_sum += gx_raw;
    gy_sum += gy_raw;
    gz_sum += gz_raw;
    
    delay(3);
  }
  
  // Calculate offsets
  ax_offset = (ax_sum / samples) / ACCEL_SCALE;
  ay_offset = (ay_sum / samples) / ACCEL_SCALE;
  az_offset = ((az_sum / samples) / ACCEL_SCALE) - 1.0; // -1g gravity
  
  gx_offset = (gx_sum / samples) / GYRO_SCALE;
  gy_offset = (gy_sum / samples) / GYRO_SCALE;
  gz_offset = (gz_sum / samples) / GYRO_SCALE;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}