// serial_configuration.ino
bool reset_cmd = false;

void enviarDatos() {
  Serial.print("DATA,");
  Serial.print(vx, 2); Serial.print(",");
  Serial.print(wz, 2); Serial.print(",");
  Serial.print(ax, 2); Serial.print(",");
  Serial.print(ay, 2); Serial.print(",");
  Serial.print(az, 2); Serial.print(",");
  Serial.print(gx, 2); Serial.print(",");
  Serial.print(gy, 2); Serial.print(",");
  Serial.print(gz, 2); Serial.print(",");
  Serial.println();
  Serial.flush();
}

void recibirDatos() {
  if (Serial.available() > 0) {
    last_serial_activity = millis();
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("CMD,")) {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      
      if (firstComma != -1 && secondComma != -1) {
        float linear = input.substring(firstComma + 1, secondComma).toFloat();
        float angular = input.substring(secondComma + 1).toFloat();

        // Cinemática Inversa
        float vI = linear - (angular * L) / 2.0;
        float vD = linear + (angular * L) / 2.0;
        Left.wRef = (2.0 * vI) / D;
        Right.wRef = (2.0 * vD) / D;
      }
    }
  }
}

void watchdog() {
  if (millis() - last_serial_activity > SERIAL_TIMEOUT_MS) {
    serial_connected = false;
    // EMERGENCIA: Detener referencias de velocidad
    Left.wRef = 0;
    Right.wRef = 0;
    // Si quieres frenado instantáneo:
    ST.motor(1, 0);
    ST.motor(2, 0);
  } else {
    serial_connected = true;
  }
}
