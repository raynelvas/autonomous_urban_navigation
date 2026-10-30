                   //______DEFINICION DE GPIO________________________________-
#define GPIO_VOLTAJE_12V 34

#define GPIO_RELE_STOP 26
#define GPIO_RELE_LEFT 27
#define GPIO_RELE_RIGHT 14
#define GPIO_RELE_SAFETY 25
#define GPIO_RELE_MOTOR 13
//___________________________________________


//_____________VARIABLES_________________________________________
float voltage_12V = 0.0;
//___________________________________________

//_____________cariables para seguridad de microros y serial______________
unsigned long last_serial_activity = 0;
const unsigned long SERIAL_TIMEOUT_MS = 2000; // 2 segundos sin actividad serial
bool serial_connected = false;
bool reset = false;

//_____________________________________________________________________

//________TIEMPO SERIAL___________________
const int sampleTime_serial = 50;  // Tiempo de muestreo milisegundos
unsigned long lastTime_serial = 0;
//_______________________________________

//________________VARIABLES GLOBALES DE MICROROS__________________
const int led_error = 2;
//_______________________________________________________

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(GPIO_VOLTAJE_12V, ADC_11db);
  delay(500);

  //___________RELES___________________
  pinMode(GPIO_RELE_MOTOR, OUTPUT);
  pinMode(GPIO_RELE_STOP, OUTPUT);
  pinMode(GPIO_RELE_LEFT, OUTPUT);
  pinMode(GPIO_RELE_RIGHT, OUTPUT);
  pinMode(GPIO_RELE_SAFETY, OUTPUT);

  digitalWrite(GPIO_RELE_MOTOR, HIGH);
  digitalWrite(GPIO_RELE_STOP, HIGH);
  digitalWrite(GPIO_RELE_LEFT, HIGH);
  digitalWrite(GPIO_RELE_RIGHT, HIGH);
  digitalWrite(GPIO_RELE_SAFETY, HIGH);

  // Opcional: espera Serial (solo si necesitas debugging tempranero)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);  // espera max 2s
  }

}

void loop() {
  //watchdog();

  if (millis() - lastTime_serial > sampleTime_serial) {
    voltage_12V = readVoltage(GPIO_VOLTAJE_12V);
    lastTime_serial = millis();
    enviarDatos(voltage_12V);
    recibirDatos();
  }
}
