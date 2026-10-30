// serial_configuration.ino
bool reset_cmd = false;

void enviarDatos() {
  Serial.print("DATA,");
  Serial.print(Right.w, 4); Serial.print(",");
  Serial.print(Left.w, 4); Serial.print(",");
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
        float right = input.substring(firstComma + 1, secondComma).toFloat();
        float left = input.substring(secondComma + 1).toFloat();

        // Cinemática Inversa
        int control_right = (int)right;
        int control_left = (int)left;

        Right.wRef = control_right; //rad/s
        Left.wRef = control_left; //rad/s
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
