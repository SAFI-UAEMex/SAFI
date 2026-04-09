/*
 * Sistema Integrado de Telemetría: MPU6050 (IMU) + BMP180 (Atmósfera)
 * Transmite 9 valores: ax, ay, az, gx, gy, gz, temp, press, alt
 */

#include <Wire.h>
#include <Adafruit_BMP085.h>

// --- Configuración MPU6050 ---
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

#define ACCEL_SCALE 16384.0  // Para ±2g
#define GYRO_SCALE  131.0    // Para ±250°/s

// Variables MPU6050
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// --- Configuración BMP180 ---
Adafruit_BMP085 bmp;

// --- Control de Tiempo ---
unsigned long lastTime = 0;
const int SAMPLE_RATE = 50; // Hz (50 lecturas por segundo)
const int SAMPLE_PERIOD = 1000 / SAMPLE_RATE; // ms

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  Wire.setClock(400000); // 400kHz I2C para lecturas rápidas
  delay(100);
  
  // Iniciar BMP180
  if (!bmp.begin()) {
    Serial.println("Error: No se encontró el BMP180. Revisa conexiones.");
    while (1) {}
  }

  // Iniciar MPU6050
  initMPU6050();
  
  // Calibrar MPU6050 (mantener quieto)
  calibrateMPU();
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= SAMPLE_PERIOD) {
    lastTime = currentTime;
    
    // 1. Leer MPU6050
    readMPU6050();
    float ax = (ax_raw / ACCEL_SCALE) - ax_offset;
    float ay = (ay_raw / ACCEL_SCALE) - ay_offset;
    float az = (az_raw / ACCEL_SCALE) - az_offset;
    float gx = (gx_raw / GYRO_SCALE) - gx_offset;
    float gy = (gy_raw / GYRO_SCALE) - gy_offset;
    float gz = (gz_raw / GYRO_SCALE) - gz_offset;
    
    // 2. Leer BMP180
    float temperature = bmp.readTemperature();
    int32_t pressure = bmp.readPressure();
    float altitude = bmp.readAltitude();
    
    // 3. Enviar cadena CSV unificada (9 valores)
    Serial.print(ax, 4); Serial.print(",");
    Serial.print(ay, 4); Serial.print(",");
    Serial.print(az, 4); Serial.print(",");
    Serial.print(gx, 4); Serial.print(",");
    Serial.print(gy, 4); Serial.print(",");
    Serial.print(gz, 4); Serial.print(",");
    Serial.print(temperature, 2); Serial.print(",");
    Serial.print(pressure); Serial.print(",");
    Serial.println(altitude, 2);
  }
}

void initMPU6050() {
  writeRegister(PWR_MGMT_1, 0x00); delay(100);
  writeRegister(CONFIG, 0x03);     // DLPF ~44Hz
  writeRegister(GYRO_CONFIG, 0x00); // ±250 °/s
  writeRegister(ACCEL_CONFIG, 0x00); // ±2g
  delay(100);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  
  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Saltar temperatura del MPU
  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();
}

void calibrateMPU() {
  const int samples = 500;
  long ax_s = 0, ay_s = 0, az_s = 0, gx_s = 0, gy_s = 0, gz_s = 0;
  
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    ax_s += ax_raw; ay_s += ay_raw; az_s += az_raw;
    gx_s += gx_raw; gy_s += gy_raw; gz_s += gz_raw;
    delay(4);
  }
  
  ax_offset = (ax_s / samples) / ACCEL_SCALE;
  ay_offset = (ay_s / samples) / ACCEL_SCALE;
  az_offset = ((az_s / samples) / ACCEL_SCALE) - 1.0;
  gx_offset = (gx_s / samples) / GYRO_SCALE;
  gy_offset = (gy_s / samples) / GYRO_SCALE;
  gz_offset = (gz_s / samples) / GYRO_SCALE;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg); Wire.write(value);
  Wire.endTransmission(true);
}