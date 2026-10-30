//voltage.ino
#define ADC_RESOLUTION 4095.0
#define VREF 3.3

// Calibración basada en mediciones reales
// Punto 1: ADC=717 → 3.45V
// Punto 2: ADC=2921 → 12.21V
#define ADC_PUNTO_BAJO 717.0
#define ADC_PUNTO_ALTO 2921.0
#define VOLTAJE_PUNTO_BAJO 3.45
#define VOLTAJE_PUNTO_ALTO 12.21

float voltajeFiltrado = 12.0;  // Variable para el filtro (mejor nombre)

float readVoltage(int GPIO){
  int rawValue = analogRead(GPIO);
  
  // Para ADC=0, retornar 0 directamente
  if (rawValue == 0) {
    voltajeFiltrado = 0.0;  // Reiniciar filtro
    return 0.0;
  }
  
  // Cálculo de la pendiente (m) y punto de corte (b)
  float m = (VOLTAJE_PUNTO_ALTO - VOLTAJE_PUNTO_BAJO) / (ADC_PUNTO_ALTO - ADC_PUNTO_BAJO);
  float b = VOLTAJE_PUNTO_BAJO - m * ADC_PUNTO_BAJO;
  
  float voltageIn = m * rawValue + b;

  // Filtro pasa-bajos: 90% valor anterior + 10% lectura nueva
  // Esto elimina ruido sin afectar la medición de batería que cambia lentamente
  voltajeFiltrado = 0.9 * voltajeFiltrado + 0.1 * voltageIn;  
  return voltajeFiltrado;
}
