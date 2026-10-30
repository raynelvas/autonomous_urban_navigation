// imu_reader.ino  (MPU6500 sin librerías)

#define MPU_ADDR 0x68

// conversion constants
const float G_TO_MS2 = 9.80665f;
const float ACCEL_SCALE = 16384.0;   // ±2g
const float GYRO_SCALE  = 131.0;     // ±250°/s

// bias del giroscopio
float gyro_bias_x = 0.0;
float gyro_bias_y = 0.0;
float gyro_bias_z = 0.0;

bool IMU_begin() {

  Wire.begin();
  delay(200);

  const int maxAttempts = 5;
  int attempts = 0;

  while (attempts < maxAttempts) {

    // Wake up MPU
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) break;

    attempts++;
    delay(200);
  }

  if (attempts >= maxAttempts) {
    return false;
  }

  // Gyro ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Accel ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  // DLPF ≈ 44Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  // Sample rate ≈ 200Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(4);
  Wire.endTransmission();

  // pequeña espera para estabilizar sensor
  delay(500);

  // ---------- CALIBRACIÓN DEL GIROSCOPIO ----------
  const int samples = 1000;

  long sum_x = 0;
  long sum_y = 0;
  long sum_z = 0;

  for (int i = 0; i < samples; i++) {

    uint8_t buffer[6];

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // registro gyro X
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);

    if (Wire.available() == 6) {

      buffer[0] = Wire.read();
      buffer[1] = Wire.read();
      buffer[2] = Wire.read();
      buffer[3] = Wire.read();
      buffer[4] = Wire.read();
      buffer[5] = Wire.read();

      int16_t gx_raw = (buffer[0] << 8) | buffer[1];
      int16_t gy_raw = (buffer[2] << 8) | buffer[3];
      int16_t gz_raw = (buffer[4] << 8) | buffer[5];

      sum_x += gx_raw;
      sum_y += gy_raw;
      sum_z += gz_raw;
    }

    delay(2);
  }

  gyro_bias_x = (sum_x / (float)samples) / GYRO_SCALE * DEG_TO_RAD;
  gyro_bias_y = (sum_y / (float)samples) / GYRO_SCALE * DEG_TO_RAD;
  gyro_bias_z = (sum_z / (float)samples) / GYRO_SCALE * DEG_TO_RAD;

  return true;
}

void IMU_update() {

  uint8_t buffer[14];

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() != 14) return;

  for (int i = 0; i < 14; i++) {
    buffer[i] = Wire.read();
  }

  int16_t accelX = (buffer[0] << 8) | buffer[1];
  int16_t accelY = (buffer[2] << 8) | buffer[3];
  int16_t accelZ = (buffer[4] << 8) | buffer[5];

  int16_t gyroX = (buffer[8] << 8) | buffer[9];
  int16_t gyroY = (buffer[10] << 8) | buffer[11];
  int16_t gyroZ = (buffer[12] << 8) | buffer[13];

  // --------- ACELERACIÓN m/s² ----------
  ax = (accelX / ACCEL_SCALE) * G_TO_MS2;
  ay = (accelY / ACCEL_SCALE) * G_TO_MS2;
  az = (accelZ / ACCEL_SCALE) * G_TO_MS2;

  // --------- GIROSCOPIO rad/s ----------
  gx = (gyroX / GYRO_SCALE) * DEG_TO_RAD;
  gy = (gyroY / GYRO_SCALE) * DEG_TO_RAD;
  gz = (gyroZ / GYRO_SCALE) * DEG_TO_RAD;

  // quitar bias calibrado
  gx -= gyro_bias_x;
  gy -= gyro_bias_y;
  gz -= gyro_bias_z;

  // deadband igual que tu código original
  if (abs(gx) < 0.0025) gx = 0;
  if (abs(gy) < 0.0025) gy = 0;
  if (abs(gz) < 0.0025) gz = 0;
}
