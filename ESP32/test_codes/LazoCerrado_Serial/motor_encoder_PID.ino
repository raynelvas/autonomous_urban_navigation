//motor_encoder.ino
unsigned int ppr = 4000;         // Número de muescas que tiene el disco del encoder.

void beginEncoder(int LA,int LB,int LZ,int RA,int RB,int RZ){
  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Opcional
  
  encoderR.attachFullQuad(RA, RB);  // ✅ Configura el encoder global derecho
  encoderR.setCount(0);             // ✅ Inicializa el contador
  
  encoderL.attachFullQuad(LA, LB);  // ✅ Configura el encoder global izquierdo  
  encoderL.setCount(0);             // ✅ Inicializa el contador

  pinMode(LZ, INPUT_PULLUP);

  pinMode(RZ, INPUT_PULLUP);
  Serial.println("Encoder Omron E6B2-CWZ3E listo.");
}

void beginPid(){
  /*Matlab Derecho: 
   *   Kc = 0.025224 / 0.025689 / 0.025399
   *   Ti = 0.002533 s / 0.005
   *   Td = 0.000000 s
   */
  motorR.setGains (0.024224, 0.005, 0.0); // (Kc,Ti,Td)

  /*Matlab Izquierdo:
   *   Kc = 0.027532 / 0.027923 / 0.028193
   *   Ti = 0.005 s / 0.005
   *   Td = 0.000000 s
   */
  //motorL.setGains (0.026532, 0.005, 0.0); // (Kc,Ti,Td); 
  motorL.setGains (0.026532, 0.005, 0.0);; // (Kc,Ti,Td);
    
  motorL.setCvLimits(63, 0);
  motorR.setCvLimits(63, 0);

  motorL.setPvLimits(12.03, 0); 
  motorR.setPvLimits(12.71 , 0); 
  
  Serial.println("Controlador PID configurado y listo.");
}

void encoderPID(){
  updateMotor(Left, encoderL, motorL, 1);
  updateMotor(Right, encoderR, motorR, 2);
}

void updateMotor(DatosMotores &m, ESP32Encoder &enc, motorControl &ctrl, int id) {
  const float alpha = 0.95;
  long count = enc.getCount();
  enc.setCount(0);
  
  m.wSinFilter = count * (2 * PI * 1000) / (dt_enc * ppr);
  m.w = alpha * m.wSinFilter + (1-alpha) * m.wAnterior; // velocidad angular actual en rad/s
  m.wAnterior = m.w;
  
  m.outValue = ctrl.compute(m.wRef, m.w);
  ST.motor(id, m.outValue);
}
