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
  //Serial.println("Encoder Omron E6B2-CWZ3E listo.");
}

void beginPid(){
  motorL.setGains (0.026532, 0.005, 0.0); // (Kc,Ti,Td);
  motorR.setGains (0.024224, 0.005, 0.0); // (Kc,Ti,Td)

  motorL.setCvLimits(63, 0);
  motorR.setCvLimits(63, 0);

  motorL.setPvLimits(12.03, 0); 
  motorR.setPvLimits(12.71 , 0); 
  
  //Serial.println("Controlador PID configurado y listo.");
}

void encoderPID(){
  //Serial.print("IZQ: ");
  updateMotor(Left, encoderL, motorL, 1);

  //Serial.print("DER: "); 
  updateMotor(Right, encoderR, motorR, 2);

  vx = (Right.v + Left.v) / 2;
  wz = (Right.v - Left.v) / L ; // rad/s

  // Filtro sencillo deadzone
  if (abs(vx) < 0.06) vx = 0;
  if (abs(wz) < 0.08) wz = 0;
}

void updateMotor(DatosMotores &m, ESP32Encoder &enc, motorControl &ctrl, int id) {
  const float alpha = 0.95;
  long count = enc.getCount();
  enc.setCount(0);
  
  m.wSinFilter = count * (2 * PI * 1000) / (dt_enc * ppr);
  m.w = alpha * m.wSinFilter + (1-alpha) * m.wAnterior; // velocidad angular actual en rad/s
  m.wAnterior = m.w;
  
  m.v = (m.w * D) / 2;
  m.outValue = ctrl.compute(m.wRef, m.w);
  ST.motor(id, m.outValue);
  /*Serial.print(m.wRef);
  Serial.print(" / ");
  Serial.print(m.w);
  Serial.print(" / ");
  Serial.println(m.outValue);
  Serial.println();*/
}
