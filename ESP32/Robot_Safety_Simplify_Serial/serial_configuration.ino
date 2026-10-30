// serial_configuration.ino
void enviarDatos(float V12) {
  // Usar buffer para reducir fragmentación de strings
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "DATA,%.2f,\n", V12);
  Serial.print(buffer);
  //Serial.flush();
}

void recibirDatos() {
  if (Serial.available() > 0) {
    last_serial_activity = millis();
    serial_connected = true;
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("CMD,")) {
      int firstComma = input.indexOf(',');
      
      if (firstComma != -1) {
        String rele_str = input.substring(firstComma + 1);
        
        uint8_t rele_cmd = (uint8_t)rele_str.toInt();
        /*Serial.println("MOTOR: " + String(rele_cmd & (1 << 0)));
        Serial.println("STOP: " + String(rele_cmd & (1 << 1)));
        Serial.println("LEFT: " + String(rele_cmd & (1 << 2)));
        Serial.println("RIGHT: " + String(rele_cmd & (1 << 3)));
        Serial.println("SAFETY: " + String(rele_cmd & (1 << 4)));
        Serial.println("");*/

        
        digitalWrite(GPIO_RELE_STOP,  rele_cmd & (1 << 0) ? LOW : HIGH);  
        digitalWrite(GPIO_RELE_LEFT,     rele_cmd & (1 << 1) ? LOW : HIGH);  
        digitalWrite(GPIO_RELE_RIGHT,     rele_cmd & (1 << 2) ? LOW : HIGH);  
        digitalWrite(GPIO_RELE_SAFETY,    rele_cmd & (1 << 3) ? LOW : HIGH);  
        digitalWrite(GPIO_RELE_MOTOR,   rele_cmd & (1 << 4) ? LOW : HIGH);
        
      }
    }
  }
}

void watchdog() {
  if (millis() - last_serial_activity > SERIAL_TIMEOUT_MS) {
    serial_connected = false;
    digitalWrite(GPIO_RELE_MOTOR, HIGH);
    digitalWrite(GPIO_RELE_STOP, HIGH);
    digitalWrite(GPIO_RELE_LEFT, HIGH);
    digitalWrite(GPIO_RELE_RIGHT, HIGH);
    digitalWrite(GPIO_RELE_SAFETY, HIGH);
  } else {
    serial_connected = true;
  }
}
