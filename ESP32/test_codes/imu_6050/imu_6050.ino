
//_______VARIABLES IMU_________________________________
float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;

unsigned long lastTime_imu = 0, dt_imu = 0;
float yaw_imu = 0.0, roll_imu = 0.0, pitch_imu = 0.0;
//______________________________________________________


void setup() {
  Serial.begin(115200);  // Inicializa el puerto serial a 9600 bps
  Serial.println("ESP32 iniciada y lista para recibir comandos");

  while (!Serial) {}

  if (!IMU_begin()) {
    Serial.println("Error al inicializar el IMU");
    while (1);
  }

  lastTime_imu = millis();
}

void loop() {
  dt_imu = millis() - lastTime_imu;
  if (dt_imu >= 100) {
    lastTime_imu = millis();
    IMU_update();
  }
  delay(10);
}
