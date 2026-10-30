#include "SabertoothSimplified.h"

SabertoothSimplified::SabertoothSimplified(Stream &port) : serial(port) {}

void SabertoothSimplified::motor(int motor, int power) {
  if (motor < 1 || motor > 2) return;
  power = constrain(power, -63, 63);  // Rango de -63 a +63

  byte value;
  
  if (motor == 1) {
    // Motor 1: valores entre 1 y 127
    // -63 → 1, 0 → 64, +63 → 127
    value = map(power, -63, 63, 1, 127);
  } else {
    // Motor 2: valores entre 128 y 255  
    // -63 → 128, 0 → 192, +63 → 255
    value = map(power, -63, 63, 128, 255);
  }
  
  sendByte(value);
}

void SabertoothSimplified::sendByte(byte value) {
  serial.write(value);
}
