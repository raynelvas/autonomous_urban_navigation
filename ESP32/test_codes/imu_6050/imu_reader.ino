#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial); // Espera a que el puerto serial esté listo (solo para placas con USB nativo)
  
  Serial.println("Escaneando bus I2C...");
  Wire.begin();
}

void loop() {
  byte error, address;
  int dispositivosEncontrados = 0;

  Serial.println("Escaneando...");
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en direccion 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      dispositivosEncontrados++;
    }
  }

  if (dispositivosEncontrados == 0) {
    Serial.println("No se encontraron dispositivos I2C");
  } else {
    Serial.print("Escaneo completado. ");
    Serial.print(dispositivosEncontrados);
    Serial.println(" dispositivo(s) encontrado(s).");
  }

  Serial.println("----------------------------------------");
  delay(5000); // Espera 5 segundos antes del siguiente escaneo
}
