/*
 * Arduino + MPU6050 IMU Sensor + SD Card Logger + LED Indicator
 * High-frequency data logging for Python visualization
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// --- MPU6050 Configuration ---
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

#define ACCEL_SCALE 16384.0  // Para ±2g
#define GYRO_SCALE  131.0    // Para ±250°/s

// --- SD Card Configuration ---
const int chipSelect = 4; // Pin CS del módulo SD
File dataFile;
int flushCounter = 0;     // Contador para guardar físicamente en la SD

// --- LED Indicator ---
const int LED_PIN = 8;    // Pin donde conectarás el LED

// --- Timing ---
unsigned long lastTime = 0;
const int SAMPLE_RATE = 100; // Hz
const int SAMPLE_PERIOD = 1000 / SAMPLE_RATE; // ms

// --- Raw sensor data ---
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t temperature_raw;

// --- Calibration offsets ---
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

void setup() {
  Serial.begin(115200);
  
  // Configurar el pin del LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Apagado al iniciar
  
  // Inicialización I2C
  Wire.begin(); 
  Wire.setClock(400000); // 400kHz I2C
  delay(100);
  
  // Inicializar MPU6050
  initMPU6050();
  
  // Calibración
  Serial.println("Calibrando... ¡Mantén el sensor quieto!");
  calibrateSensor();
  Serial.println("¡Calibración completa!");
  
  // Inicializar Tarjeta SD
  Serial.print("Inicializando tarjeta SD...");
  if (!SD.begin(chipSelect)) {
    Serial.println("¡Fallo en la tarjeta SD o no está presente!");
    // Si hay error, parpadear el LED infinitamente
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  Serial.println("SD inicializada correctamente.");

  // Crear o abrir el archivo CSV
  dataFile = SD.open("IMU_DATA.csv", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error al abrir IMU_DATA.csv");
    // Parpadeo infinito si no se puede abrir el archivo
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  Serial.println("Iniciando registro de datos...");
  
  // ¡Todo listo! Encender el LED permanentemente
  digitalWrite(LED_PIN, HIGH); 
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= SAMPLE_PERIOD) {
    lastTime = currentTime;
    
    // Leer datos del sensor
    readMPU6050();
    
    // Convertir a unidades reales
    float ax = (ax_raw / ACCEL_SCALE) - ax_offset;
    float ay = (ay_raw / ACCEL_SCALE) - ay_offset;
    float az = (az_raw / ACCEL_SCALE) - az_offset;
    
    float gx = (gx_raw / GYRO_SCALE) - gx_offset;
    float gy = (gy_raw / GYRO_SCALE) - gy_offset;
    float gz = (gz_raw / GYRO_SCALE) - gz_offset;
    
    // --- Escribir en la memoria SD ---
    if (dataFile) {
      dataFile.print(ax, 4); dataFile.print(",");
      dataFile.print(ay, 4); dataFile.print(",");
      dataFile.print(az, 4); dataFile.print(",");
      dataFile.print(gx, 4); dataFile.print(",");
      dataFile.print(gy, 4); dataFile.print(",");
      dataFile.println(gz, 4);

      // Forzar el guardado físico en la SD cada 100 muestras (1 segundo)
      flushCounter++;
      if (flushCounter >= 100) {
        dataFile.flush(); 
        flushCounter = 0;
      }
    } else {
      Serial.println("Error escribiendo en el archivo SD.");
    }
  }
}

void initMPU6050() {
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);
  writeRegister(CONFIG, 0x03);
  writeRegister(GYRO_CONFIG, 0x00);
  writeRegister(ACCEL_CONFIG, 0x00);
  delay(100);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();
  
  temperature_raw = (Wire.read() << 8) | Wire.read();
  
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
  
  ax_offset = (ax_sum / samples) / ACCEL_SCALE;
  ay_offset = (ay_sum / samples) / ACCEL_SCALE;
  az_offset = ((az_sum / samples) / ACCEL_SCALE) - 1.0;
  
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