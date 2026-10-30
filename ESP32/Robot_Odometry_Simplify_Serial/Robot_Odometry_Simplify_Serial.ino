// Robot_Odometry.ino (PRINCIPAL)
#include <ESP32Encoder.h>
#include <SabertoothSimplified.h>
#include <motorControl.h>

#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;
SemaphoreHandle_t i2cMutex;

//___________ENCODER DERECHO_____________
#define RightA 27
#define RightB 14
#define RightZ 26
//_______________________________________

//__________ENCODER LeftUIERDO____________
#define LeftA 5
#define LeftB 18
#define LeftZ 19
//_______________________________________

//__________________CONTROL DE MOTORES______________________
#define SABERTOOTH_TX 17   // GPIO17 (TX2)
HardwareSerial SabertoothSerial(2);
SabertoothSimplified ST(SabertoothSerial); //El rango de velocidad es de -63 a +63, donde:
//_______________________________________

//_____________cariables para seguridad de microros y serial______________
unsigned long last_serial_activity = 0;
const unsigned long SERIAL_TIMEOUT_MS = 2000; // 2 segundos sin actividad serial
bool serial_connected = false;
bool reset = false;
//_____________________________________________________________________

//___________Parametros del robot_____________
float L = 0.84; // distancia entre ruedas metros
float D = 0.33; // Diametro de la rueda en metros
//_______________________________________________

//_______VARIABLES IMU_________________________________
float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;
//______________________________________________________

//________CONFIGURACION ENCODER__________
const int sampleTime_enc = 33.333;  // Tiempo de muestreo milisegundos
volatile unsigned long lastTime_enc = 0, dt_enc = 0;;

float vx = 0, wz = 0; //Velocidad lineal y angular del robot [m/s] y [rad/s]
//_______________________________________

//________TIEMPO SERIAL___________________
const int sampleTime_serial = 33.3333;  // Tiempo de muestreo milisegundos
unsigned long lastTime_serial = 0;
//_______________________________________

//______________ESTRUCTURA PARA DATOS MOTORES___________________________
struct DatosMotores {
  double w = 0.0;
  double wAnterior = 0.0;
  double wRef = 0.0;
  int outValue = 0;
  double wSinFilter = 0.0;
  double v = 0.0;
};
//_________________________________________________________

//____________creación de instancias__________________________
DatosMotores Left;
DatosMotores Right;

ESP32Encoder encoderL;
ESP32Encoder encoderR;

motorControl motorR(sampleTime_enc);
motorControl motorL(sampleTime_enc);
//_________________________________________________________

const int led_error = 2;

void enviarDatos(); // prototype from serial_configuration.ino
void recibirDatos();

void setup() {
  i2cMutex = xSemaphoreCreateMutex();

  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(led_error, OUTPUT);
  digitalWrite(led_error, HIGH);

  //_____________________MOTOR______________________
  SabertoothSerial.begin(38400, SERIAL_8N1, -1, SABERTOOTH_TX);
  ST.motor(1, 0); ST.motor(2, 0);

  // Opcional: espera Serial (solo si necesitas debugging tempranero)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);
  }

  //_____________________ENCODER______________________
  beginEncoder(LeftA, LeftB, LeftZ, RightA, RightB, RightZ);

  //____________IMU______________
  while (!Serial) {}

  if (!IMU_begin()) {
    Serial.println("Error al inicializar el IMU");
    while (1);
  }
  //_______________________________

  //_____________CONTROL PID_______________
  beginPid();

  //__________Tarea para control de velocidad y comunicacion serial, en Core 0_____________________
  xTaskCreatePinnedToCore(
    ControlLoop,
    "ControlLoopTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );
  //______________________________________________________________________
  last_serial_activity = millis();
  lastTime_enc = millis();
  digitalWrite(led_error, LOW);
}

void loop() {
  // watchdog revisa micro-ROS y actualiza microros_operational (no reinicia)
  watchdog();

  dt_enc = millis() - lastTime_enc;
    if (dt_enc >= sampleTime_enc) {
      lastTime_enc = millis();
      encoderPID();
  }


}

void ControlLoop(void *parameter) {
  //___________Inicializaciones de tiempo_____________
  lastTime_serial = millis();

  while (1) {
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (mpu.update()) {
        IMU_update();
      }
      xSemaphoreGive(i2cMutex); // Soltar el bus
    }

    if (millis() - lastTime_serial > sampleTime_serial) {
      lastTime_serial = millis();
      enviarDatos();
      recibirDatos();
    }
    vTaskDelay(1);



  }
}
