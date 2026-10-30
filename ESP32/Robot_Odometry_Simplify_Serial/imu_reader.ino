//imu_reader.ino
// conversion constants
const float G_TO_MS2 = 9.80665f;

bool IMU_begin() {
  Wire.begin();
  delay(2000);
  //Wire.setClock(400000);

  // ---- Configuración sugerida para robot móvil en vias (recomendación) ----
  // Explicación: para robots en vía, no esperamos G's enormes ni giros extremadamente rápidos.
  // Elegimos sensibilidad moderada para buena resolución y filtros DLPF para estabilidad.
  MPU9250Setting setting;
  // Recomendación: +-8g para aceleración (suficiente para baches fuertes) o +-4g si quieres mayor resolución.
  setting.accel_fs_sel = ACCEL_FS_SEL::A8G;        // <-- mejor resolución/ruido vs A16G
  setting.gyro_fs_sel  = GYRO_FS_SEL::G500DPS;    // <-- 500 dps es conservador; G250DPS es más sensible si no esperas giros fuertes
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;    // amortigua ruido en giros rápidos
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;  // suaviza vibraciones, mantiene respuesta decente

  // Intentar inicializar el IMU varias veces
  const int maxAttempts = 5;
  int attempts = 0;
  while (!mpu.setup(0x68, setting)) {
    attempts++;
    if (attempts >= maxAttempts) {
      //Serial.println("IMU: init failed after multiple attempts.");
      return false;
    }
    //Serial.println("IMU: init failed, retrying...");
    delay(200);
  }

  // Desactivar verbose para normal operation
  mpu.verbose(false);
  return true;
}

void IMU_update() {

  //______________ACELERACION m/s2____________
  // Aceleración (ya escalada según fullscale): getAccX/Y/Z()
  ax = mpu.getAccX() * G_TO_MS2;
  ay = mpu.getAccY() * G_TO_MS2;
  az = mpu.getAccZ() * G_TO_MS2;

  /*if (abs(ax) < 0.07) ax = 0;
    if (abs(ay) < 0.2) ay = 0;
    if (abs(az) < 0.07) az = 0;*/

  //__________VELOCIDAD ANGULAR en rad/s_____________
  gx = mpu.getGyroX() * DEG_TO_RAD;
  gy  = mpu.getGyroY() * DEG_TO_RAD;
  gz  = mpu.getGyroZ() * DEG_TO_RAD;

  if (abs(gx) < 0.0025) gx = 0;
  if (abs(gy) < 0.0025) gy = 0;
  if (abs(gz) < 0.0025) gz = 0;

}
