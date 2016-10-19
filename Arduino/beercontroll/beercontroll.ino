#include <math.h><br>

#define QUEMADOR_LICOR  10
#define QUEMADOR_MACERADOR  8
#define SENSOR_LICOR  13
#define SENSOR_MACERADOR  14
#define R_NOMINAL 100000  //Resistencia nominal
#define T_NOMINAL 25      //Temperatura nominal
#define NUM_MUESTRAS 5    // Nro. de muestras.
#define SERIESRESISTOR 4700 // Valor de Resistencias en serie.
#define TIEMPO_MUESTREO 2000 // Retardo entre medidas

// Valores para calcular el valor Beta, si no lo sabemos
float T1 = 0 + 273.15;  // [K]        Temperatura del 1º punto del test (en grados Kelvin)
float T2 = 100 + 273.15;   // [K]        Temperatura del 2º punto del test (en grados Kelvin)
float RT1 = 32701.95; // [ohms]     Resistencia a 273K grados Kelvin (0ºC)
float RT2 = 6610.10; // [ohms]     Resistencia a 373K grados Kelvin (100ºC)

double B = 3950; // Beta Termistor
int muestras[NUM_MUESTRAS];
int i;

void setup() {
  Serial.begin(115200);
  pinMode(QUEMADOR_LICOR, OUTPUT);
  pinMode(QUEMADOR_MACERADOR, OUTPUT);
  pinMode(SENSOR_LICOR, INPUT);
  pinMode(SENSOR_MACERADOR, INPUT);
  B = (log(RT1 / RT2)) / ((1 / T1) - (1 / T2));
}

int convertirTemp(float dato) {
  float media = dato;
  float temp;
  media = 1023 / media - 1; // Convert the thermal stress value to resistance
  media = SERIESRESISTOR / media;
  temp = media / R_NOMINAL;     // (R/Ro)
  temp = log(temp); // ln(R/Ro)
  temp /= B;                   // 1/B * ln(R/Ro)
  temp += 1.0 / (T_NOMINAL + 273.15); // + (1/To)
  temp = 1.0 / temp;                 // Invierto el valor;
  temp -= 273.15;                         // Convertir a ºC
  return  temp;
}

void loop() {
  float mediaLicor = 0;
  float mediaMacerador = 0;
  long tLicor;
  long tMacerador;
  bool QuemadorLicor = false;
  bool QuemadorMacerador = false;
  for (i = 0; i < NUM_MUESTRAS; i++) {
    mediaLicor += analogRead(SENSOR_LICOR);
    mediaMacerador += analogRead(SENSOR_MACERADOR);
    delay(10);
  }
  mediaLicor /= NUM_MUESTRAS;
  mediaMacerador /= NUM_MUESTRAS;
  tLicor = convertirTemp(mediaLicor);
  tMacerador = convertirTemp(mediaMacerador);
  Serial.print("{");
  Serial.print("\"licorTemp\":");
  Serial.print(tLicor);
  Serial.print(" ,");
  Serial.print("\"maceradorTemp\":");
  Serial.print(tMacerador);
  Serial.print(" ,");
  Serial.print("\"maceradorQuemador\":");
  Serial.print(QuemadorMacerador);
  Serial.print(" ,");
  Serial.print("\"licorQuemador\": ");
  Serial.print(QuemadorLicor);
  Serial.println("}");
  delay(TIEMPO_MUESTREO);
}
