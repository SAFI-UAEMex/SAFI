/*
 * Arduino + BMP180 Barometric Pressure Sensor
 * Data streaming for Python Professional Dashboard
 */

#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

// Control de tiempo para el muestreo
unsigned long lastTime = 0;
const int SAMPLE_RATE = 20; // Hz (20 lecturas por segundo es excelente para este sensor)
const int SAMPLE_PERIOD = 1000 / SAMPLE_RATE; // ms

void setup() {
  Serial.begin(115200);
  
  // Iniciar sensor
  if (!bmp.begin()) {
    Serial.println("Error: No se encontro el sensor BMP180. Revisa las conexiones.");
    while (1) {} // Detener la ejecución si no hay sensor
  }
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= SAMPLE_PERIOD) {
    lastTime = currentTime;
    
    // Leer datos del sensor
    float temperature = bmp.readTemperature(); // Grados Celsius
    int32_t pressure = bmp.readPressure();     // Pascales (Pa)
    float altitude = bmp.readAltitude();       // Metros (asumiendo presion estandar a nivel del mar)
    
    // Enviar datos en formato CSV: Temperatura, Presion, Altitud
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.println(altitude, 2);
  }
}