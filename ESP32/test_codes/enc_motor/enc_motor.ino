#include <ESP32Encoder.h>
#include <SabertoothSimplified.h>
#include <motorControl.h>

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


//===========CONTROL DE MOTORES===================
#define SABERTOOTH_TX 17   // GPIO17 (TX2)
HardwareSerial SabertoothSerial(2);
SabertoothSimplified ST(SabertoothSerial); //El rango de velocidad es de -127 a +127, donde:
//=======================================================

//==========ESTRUCTURA PARA DATOS MOTORES================
struct DatosMotores {
  double w = 0.0;  // Revoluciones por minuto calculadas.
  double wAnterior = 0.0;
  double wRef = 0.0;  // Velocidad angular en rad/s.
  int outValue = 0; //Variable de control (+-127)
  double wSinFilter = 0.0;     // Velocidad angular en rad/s.
  double v = 0.0; //Velocidad lineal de cada rueda [m/s]
};

DatosMotores Left;
DatosMotores Right;
//======================================================

//____________creación de instancias___________________________
ESP32Encoder encoderL;
ESP32Encoder encoderR;
//_______________________________________________

//________CONFIGURACION ENCODER__________
unsigned long lastTime_enc = 0, dt_enc = 0;;      // Tiempo anterior
const int sampleTime_enc = 100;  // Tiempo de muestreo milisegundos
float yaw_enc = 0.0;
//_______________________________________

//____________CONFIGURACION PID__________________
motorControl motorR(sampleTime_enc);
motorControl motorL(sampleTime_enc);
//_______________________________________________

//___________Parametros del robot_____________
float v = 0, w = 0;
float x_pos = 0.0, delta_x = 0.0; //Posición estimada por odometría METROS
float y_pos = 0.0,  delta_y = 0.0;

float L = 0.196; // distancia entre ruedas metros
float D = 0.0853; // Diametro de la rueda en metros
//____________________________________________

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
   // Esperar a que se estabilice la comunicación serial
  delay(2000);
  
  //_____________________MOTOR______________________
  SabertoothSerial.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX);
  //Serial.println("Sabertooth listo");

  //_____________________ENCODER______________________
  beginEncoder(LeftA, LeftB, LeftZ, RightA, RightB, RightZ);

  //_____________CONTROL PID_______________
  beginPid();
  
  lastTime_enc = millis();

  
  Serial.println("Left_w_actual,Right_w_actual,Left_w_ref,Right_w_ref");
}

void loop() {

  // Leer comandos del serial
  readSerialCommands();

  dt_enc = millis() - lastTime_enc; //milisegundos
  if (dt_enc >= sampleTime_enc) {// Se actualiza cada tiempo de muestreo
    lastTime_enc = millis();  // Almacenamos el tiempo actual.
    //encoderPID();
    Left.w = encoderL.getCount();
    Right.w = encoderR.getCount();
    sendToSerialPlotter();
  }
  //delay(10);
}

void readSerialCommands() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    // Procesar el comando recibido
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void processCommand(String command) {
  // Buscar la coma que separa linear y angular
  int commaIndex = command.indexOf(',');
  
  if (commaIndex > 0) {
    String linearStr = command.substring(0, commaIndex);
    String angularStr = command.substring(commaIndex + 1);
    
    // Convertir a float
    float linear = linearStr.toFloat();
    float angular = angularStr.toFloat();

    Left.wRef = linear;
    Right.wRef = angular;

    ST.motor(1, linear);
    ST.motor(2, angular);

    //float vI = linear - (angular * L) / 2;
    //float vD = linear + (angular * L) / 2;
  
    //Left.wRef = (2 * vI) / D;
    //Right.wRef = (2 * vD) / D;
    
    /*Serial.print("Comando recibido - Linear: ");
    Serial.print(linear);
    Serial.print(", Angular: ");
    Serial.println(angular);
    
  } else {
    Serial.println("Formato incorrecto. Use: linear,angular");
    Serial.println("Ejemplo: 0.5,0.3");*/
  }
}

void sendToSerialPlotter() {
  // Formato para Serial Plotter: valores separados por comas
  Serial.print(Left.w);      // Velocidad real izquierda
  Serial.print(",");
  Serial.print(Right.w);     // Velocidad real derecha
  Serial.print(",");
  Serial.print(Left.wRef);   // Velocidad referencia izquierda
  Serial.print(",");
  Serial.println(Right.wRef); // Velocidad referencia derecha

  if(digitalRead(LeftZ) == LOW){
    Serial.println();
    Serial.println("IZQUIERDO");
    Serial.println();
  }

  if(digitalRead(RightZ) == LOW){
    Serial.println();
    Serial.println("DERECHO");
    Serial.println();
  }
  
  // También puedes imprimir en formato legible para el Monitor Serial normal
  /*
  Serial.print("Left - Real: ");
  Serial.print(Left.w);
  Serial.print(" rad/s, Ref: ");
  Serial.print(Left.wRef);
  Serial.print(" rad/s | Right - Real: ");
  Serial.print(Right.w);
  Serial.print(" rad/s, Ref: ");
  Serial.println(Right.wRef);
  */
}
