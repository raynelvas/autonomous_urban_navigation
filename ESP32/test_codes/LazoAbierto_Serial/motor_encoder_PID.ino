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

void updateMotor(DatosMotores &m, ESP32Encoder &enc) {
  const float alpha = 0.05;
  long count = enc.getCount();
  enc.setCount(0);
  
  m.wSinFilter = count * (2 * PI * 1000) / (dt_enc * ppr);
  m.w = alpha * m.wSinFilter + (1-alpha) * m.wAnterior;
  m.wAnterior = m.w;
}
