//LazoCerrado.ino (PRINCIPAL)
#include <odometry.h>
#include <ESP32Encoder.h>
#include <SabertoothSimplified.h>
#include <motorControl.h>

//___________ENCODER DERECHO_____________
#define RightA 27
#define RightB 14
#define RightZ 26
//_______________________________________

//__________ENCODER IZQUIERDO____________
#define LeftA 5
#define LeftB 18
#define LeftZ 19
//_______________________________________

//________CONFIGURACION ENCODER__________
unsigned long lastTime_enc = 0, dt_enc = 0;;      // Tiempo anterior
const int sampleTime_enc = 30;  // Tiempo de muestreo milisegundos

const unsigned long SERIAL_TIMEOUT_MS = 2000; // 2 segundos sin actividad serial
const int sampleTime_serial = 30;  // Tiempo de muestreo milisegundos
unsigned long lastTime_serial = 0;
unsigned long last_serial_activity = 0;
bool serial_connected = false;
//_______________________________________

//____________creación de instancias___________________________
Odometry odometry;

ESP32Encoder encoderL;
ESP32Encoder encoderR;

motorControl motorR(sampleTime_enc);
motorControl motorL(sampleTime_enc);
//_________________________________________________________


//===========CONTROL DE MOTORES===================
#define SABERTOOTH_TX 17   // GPIO17 (TX2)
HardwareSerial SabertoothSerial(2);
SabertoothSimplified ST(SabertoothSerial); //El rango de velocidad es de -63 a +63, donde:
//=======================================================

//==========ESTRUCTURA PARA DATOS MOTORES================
struct DatosMotores {
  double w = 0.0;  // Velocidad angular en rad/s.
  double wAnterior = 0.0;
  double wSinFilter = 0.0;     // Velocidad angular cruda en rad/s.
  double wRef = 0.0;  // Velocidad angular de referencia en rad/s.
  int outValue = 0; // Variable de control (+-63)
  double v = 0.0; // Velocidad lineal de cada rueda [m/s]
};

DatosMotores Left;
DatosMotores Right;
//======================================================


void setup() {
  Serial.begin(230400);  // Inicializa el puerto serial a 9600 bps
  analogReadResolution(12);

  //_____________________MOTOR______________________
  SabertoothSerial.begin(38400, SERIAL_8N1, -1, SABERTOOTH_TX);
  ST.motor(1, 0);
  ST.motor(2, 0);

  //_____________________ENCODER______________________
  beginEncoder(LeftA, LeftB, LeftZ, RightA, RightB, RightZ);

  //_____________CONTROL PID_______________
  beginPid();


  //__________Tarea para control de velocidad, en Core 0
  xTaskCreatePinnedToCore(
    ControlLoop,        // Función
    "ControlLoopTask",  // Nombre
    4096,               // Stack size
    NULL,               // Parametros
    1,                  // Prioridad
    NULL,               // Handle
    0                   // Core 0
  );

  lastTime_enc = millis();

}

void loop() {
  watchdog();

  dt_enc = millis() - lastTime_enc; //milisegundos
  if (dt_enc >= sampleTime_enc) {// Se actualiza cada tiempo de muestreo
    lastTime_enc = millis();  // Almacenamos el tiempo actual.
    encoderPID(); // actualiza
  }

}

//_______FUNCION PARA EJECUTAR EN EL SEGUNDO NUCLEO_____________-
void ControlLoop(void *parameter) {

  while (1) {
    if (millis() - lastTime_serial > sampleTime_serial) {
      lastTime_serial = millis();
      enviarDatos();
      recibirDatos();
    }
    vTaskDelay(1);

  }

}
