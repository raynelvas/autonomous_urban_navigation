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
  motorL.setGains (0.7, 0.11, 0.03); // (Kc,Ti,Td)
  motorR.setGains (0.4, 0.11, 0.03); // (Kc,Ti,Td);

  motorL.setCvLimits(127, 0);
  motorR.setCvLimits(127, 0);

  motorL.setPvLimits(16.9, 0); //Leftuierdo 16.4 rad/s
  motorR.setPvLimits(16.9, 0); //derecho
  
  //Serial.println("Controlador PID configurado y listo.");
}

void encoderPID(){
  updateMotor(Left, encoderL, motorL, 1);
  updateMotor(Right, encoderR, motorR, 2);

  v = (Right.v + Left.v) / 2;
  w = (Right.v - Left.v) / L ; // rad/s

  //calculate current position of the robot

  yaw_enc += w * dt_enc / 1000; //radians

  float cos_h = cos(yaw_enc);
  float sin_h = sin(yaw_enc);
  delta_x = v * cos_h * dt_enc / 1000; //m
  delta_y = v * sin_h * dt_enc / 1000; //m

  x_pos += delta_x;//*0.85;
  y_pos += delta_y;//*1.1;
}

void updateMotor(DatosMotores &m, ESP32Encoder &enc, motorControl &ctrl, int id) {
  const float alpha = 0.8;
  long count = enc.getCount();
  //enc.setCount(0);
  
  m.wSinFilter = count * (2 * PI * 1000) / (dt_enc * ppr);
  m.w = alpha * m.wSinFilter + (1-alpha) * m.wAnterior;
  m.wAnterior = m.w;
  
  m.v = (m.w * D) / 2;
  m.outValue = ctrl.compute(m.wRef, m.w);
  ST.motor(id, m.outValue);
}
