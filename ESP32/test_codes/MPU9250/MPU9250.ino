// Sketch: MPU9250 - calibracion en startup + bias/scale mag preestablecido
// Basado en hideakitai/MPU9250
#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;

// ---- Valores de calibración del magnetómetro (segunda calibración, en mG) ----
const float MAG_BIAS_X = 295.85f;
const float MAG_BIAS_Y = 537.51f;
const float MAG_BIAS_Z = 199.37f;

const float MAG_SCALE_X = 1.23f;
const float MAG_SCALE_Y = 0.67f;
const float MAG_SCALE_Z = 1.43f;

// Declinación magnética
// -5° 4'  => -5.0666667 grados (negativa = WEST)
const float MAG_DECLINATION_DEG = -5.0666667f;

// Frecuencia a la que imprimimos (ms)
const unsigned long PRINT_PERIOD_MS = 500; // 10 Hz

unsigned long last_print = 0;

// conversion constants
const float G_TO_MS2 = 9.80665f;


void eulerToQuat(float roll_rad, float pitch_rad, float yaw_rad, float &qw, float &qx, float &qy, float &qz) {
  // roll (X), pitch (Y), yaw (Z) -> quaternion (w,x,y,z)
  float cy = cosf(yaw_rad * 0.5f);
  float sy = sinf(yaw_rad * 0.5f);
  float cp = cosf(pitch_rad * 0.5f);
  float sp = sinf(pitch_rad * 0.5f);
  float cr = cosf(roll_rad * 0.5f);
  float sr = sinf(roll_rad * 0.5f);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

// rotate a 3D vector around Z by angle (radians). Positive angle = CCW (right-hand)
// We need a rotation of -90deg -> pass angle = -PI/2
void rotateZ(float xin, float yin, float zin, float angle, float &xout, float &yout, float &zout) {
  float c = cosf(angle);
  float s = sinf(angle);
  xout = c * xin - s * yin;
  yout = s * xin + c * yin;
  zout = zin;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("Starting MPU9250 init...");
  Wire.begin();
  delay(2000);

  // ---- Configuración sugerida para robot móvil en vias (recomendación) ----
  // Explicación: para robots en vía, no esperamos G's enormes ni giros extremadamente rápidos.
  // Elegimos sensibilidad moderada para buena resolución y filtros DLPF para estabilidad.
  MPU9250Setting setting;
  // Recomendación: +-8g para aceleración (suficiente para baches fuertes) o +-4g si quieres mayor resolución.
  setting.accel_fs_sel = ACCEL_FS_SEL::A8G;        // <-- mejor resolución/ruido vs A16G
  setting.gyro_fs_sel  = GYRO_FS_SEL::G500DPS;    // <-- 500 dps es conservador; G250DPS es más sensible si no esperas giros fuertes
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;    // amortigua ruido en giros rápidos
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;  // suaviza vibraciones, mantiene respuesta decente

  if (!mpu.setup(0x68, setting)) {
    Serial.println("MPU connection failed. Recheck wiring and address.");
    while (1) {
      delay(1000);
    }
  }

  // Habilitar mensajes verbosos en calibración
  mpu.verbose(true);

  // ---- Calibración de ACCEL + GYRO en el arranque (se ejecuta SIEMPRE) ----
  Serial.println("Accel/Gyro calibration will start in 5 sec. Keep the device still.");
  delay(5000);
  Serial.println("Calibrating accel+gyro ... (mantener el dispositivo quieto)");
  mpu.calibrateAccelGyro(); // bloquea hasta terminar
  Serial.println("Accel+Gyro calibration finished.");

  // ---- Aplicar valores preestablecidos del MAGNETÓMETRO (no recalibramos cada inicio) ----
  Serial.println("Applying preset magnetometer bias/scale from saved calibration...");
  mpu.setMagBias(MAG_BIAS_X, MAG_BIAS_Y, MAG_BIAS_Z);   // unidades: mG (tal como tu salida)
  mpu.setMagScale(MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z);

  // ---- Declinación magnética (para convertir heading magnético -> verdadero) ----
  mpu.setMagneticDeclination(MAG_DECLINATION_DEG);

  // ---- Configurar filtro de orientación (usar cuaternión AHRS) ----
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(10); // sugerido por ti
  mpu.ahrs(true);              // asegurar que AHRS está activo

  // Desactivar verbose para normal operation
  mpu.verbose(false);

  Serial.println("MPU init done. Ready.");
}

void loop() {
  // actualizar datos y consultar si hay nuevos datos disponibles
  if (mpu.update()) {
    unsigned long now = millis();
    if (now - last_print >= PRINT_PERIOD_MS) {
      last_print = now;

      // --- Lecturas principales ---
      // Aceleración (ya escalada según fullscale): getAccX/Y/Z()
      // unidades: 'g' escaladas según library
      float ax_g = mpu.getAccX(); // [g]
      float ay_g = mpu.getAccY();
      float az_g = mpu.getAccZ();

      // Giroscopio (deg/s)
      float gx_dps = mpu.getGyroX(); // [deg/s]
      float gy_dps = mpu.getGyroY();
      float gz_dps = mpu.getGyroZ();

      // Magnetómetro (raw de librería, en mG luego de aplicar bias/scale si se usa internamente)
      float magx_raw = mpu.getMagX();
      float magy_raw = mpu.getMagY();
      float magz_raw = mpu.getMagZ();

      // Si quieres calcular el valor corregido manualmente usando tus constantes:
      /*float magx_corr = (magx_raw - MAG_BIAS_X) * MAG_SCALE_X;
        float magy_corr = (magy_raw - MAG_BIAS_Y) * MAG_SCALE_Y;
        float magz_corr = (magz_raw - MAG_BIAS_Z) * MAG_SCALE_Z;

        // Magnitud del campo magnetico (mG)
        float mag_mag_mG = sqrt(magx_corr*magx_corr + magy_corr*magy_corr + magz_corr*magz_corr);*/

      // --- Convertir unidades ---
      // Acc en m/s^2
      float ax_ms2 = ax_g * G_TO_MS2;
      float ay_ms2 = ay_g * G_TO_MS2;
      float az_ms2 = az_g * G_TO_MS2;

      // Gyro en rad/s
      float gx_rads = gx_dps * DEG_TO_RAD;
      float gy_rads = gy_dps * DEG_TO_RAD;
      float gz_rads = gz_dps * DEG_TO_RAD;


      // Cuaternión (unitario)
      /*float qw = mpu.getQuaternionW();
        float qx = mpu.getQuaternionX();
        float qy = mpu.getQuaternionY();
        float qz = mpu.getQuaternionZ();

        // Comprobación de normalización
        float qnorm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);*/

      // --- ORIENTACIÓN: ajustar yaw -90° y regenerar cuaternión ---
      // La librería ya da roll/pitch/yaw en grados (rpy)
      float roll_deg = -mpu.getRoll();
      float pitch_deg = -mpu.getPitch();
      float yaw_deg = -mpu.getYaw(); // ya con declinación aplicada

      // Ajuste de marco: rotar -90° alrededor de Z
      float yaw_deg_enu = yaw_deg + 90.0f;

      // normalizar a [-180,180)
      while (yaw_deg_enu >= 180.0f) yaw_deg_enu -= 360.0f;
      while (yaw_deg_enu < -180.0f) yaw_deg_enu += 360.0f;

      // convertir a rad
      float roll_rad = roll_deg * DEG_TO_RAD;
      float pitch_rad = pitch_deg * DEG_TO_RAD;
      float yaw_rad_enu = yaw_deg_enu * DEG_TO_RAD;

      float qw, qx, qy, qz;
      eulerToQuat(roll_rad, pitch_rad, yaw_rad_enu, qw, qx, qy, qz);

      // comprobación norma
      float qnorm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);

      /* -- - IMPRIMIR TODO (listo para ROS2) -- -*/
      /*Serial.println("---- IMU sample (frame: ENU) ----");

      // Acc en m/s^2 ENU
      Serial.printf("Acc [m/s^2]: %.4f, %.4f, %.4f\n", ax_ms2, ay_ms2, az_ms2);

      // Gyro en rad/s ENU
      Serial.printf("Gyro [rad/s]: %.4f, %.4f, %.4f\n", gx_rads, gy_rads, gz_rads);

      // Mag en mG ENU (si quieres convertir a Tesla: *1e-7)
      //Serial.printf("Mag ENU [mG] raw: %.2f, %.2f, %.2f | |B|=%.2f mG\n", mag_x_enu, mag_y_enu, mag_z_enu, mag_mag_mG);

      // Quat en ENU (w,x,y,z)
      Serial.printf("Quat ENU: w %.6f x %.6f y %.6f z %.6f  | norm=%.4f\n", qw, qx, qy, qz, qnorm);

      // Euler ENU (deg) -- coherente con cuat
      float yaw_enu_deg = yaw_deg_enu;
      Serial.printf("Euler ENU [deg]: roll %.2f pitch %.2f yaw %.2f\n", roll_deg, pitch_deg, yaw_enu_deg);

      Serial.printf("Euler ENU [deg]: roll %.2f pitch %.2f yaw %.2f\n", roll_deg, pitch_deg, yaw_deg);

      Serial.println("----------------------------------");*/
      Serial.printf("Mag: %.4f, %.4f, %.4f\n", magx_raw,magy_raw, magz_raw);
    }
  }
}
