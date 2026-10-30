#ifndef SABERTOOTHSIMPLIFIED_H
#define SABERTOOTHSIMPLIFIED_H

#include <Arduino.h>

class SabertoothSimplified {
public:
  SabertoothSimplified(Stream &port);
  void motor(int motor, int power);

private:
  Stream &serial;
  void sendByte(byte value);
};

#endif

