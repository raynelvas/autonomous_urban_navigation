#define SENSOR_PIN 34    // Pin analógico de la ESP32 (usa 32, 33, 34, 35, 36 o 39)
float voltage = 0.0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // Configura ADC de 12 bits
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);
  delay(1000);
  Serial.println("Lectura de voltaje con sensor FZ0430 iniciada...");
}

void loop() {
  voltage = readVoltage(SENSOR_PIN);

  delay(500);
}
