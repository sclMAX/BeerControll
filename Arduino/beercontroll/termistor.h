#ifndef TERMITORNTC_H
#define TERMITORNTC_H
#include <math.h><br>

class TermistorNtc
{
public:
  TermistorNtc();
  void init(int); //Inicializa el sensor de Temperatura en el pin indicado
  float getTemp();
  void update();

private:
  static const long R_NOMINAL = 100000;    //Resistencia nominal
  static const float T_NOMINAL = 25;       //Temperatura nominal
  static const int NUM_MUESTRAS = 5;       // Nro. de muestras.
  static const long SERIESRESISTOR = 4700; // Valor de Resistencias en serie.
  static const float T1 = 0 + 273.15;      // [ºK]  0ºC = 273.15ºK    Temperatura del 1º punto del test (en grados Kelvin)
  static const float T2 = 100 + 273.15;    // [ºK]   100ºC = 373.15ºK   Temperatura del 2º punto del test (en grados Kelvin)
  static const float RT1 = 32701.95;       // [ohms]    Resistencia a 273K grados Kelvin (0ºC)
  static const float RT2 = 6610.10;        // [ohms]    Resistencia a 373K grados Kelvin (100ºC)
  int Pin;
  float temperatura;
  float B = 3950; // Beta Termistor
  float calcularTemp(float);
  void calcularBeta();
  void setPin(int);
};

TermistorNtc::TermistorNtc(){};

void TermistorNtc::init(int pin)
{
  this->setPin(pin);
  pinMode(this->Pin, INPUT);
  this->calcularBeta();
}

void TermistorNtc::setPin(int pin)
{
  this->Pin = pin;
}

float TermistorNtc::calcularTemp(float medida)
{
  float media = medida;
  float temp;
  media = 1023 / media - 1; // Convert the thermal stress value to resistance
  media = this->SERIESRESISTOR / media;
  temp = media / this->R_NOMINAL;           // (R/Ro)
  temp = log(temp);                         // ln(R/Ro)
  temp /= this->B;                          // 1/B * ln(R/Ro)
  temp += 1.0 / (this->T_NOMINAL + 273.15); // + (1/To)
  temp = 1.0 / temp;                        // Invierto el valor;
  temp -= 273.15;                           // Convertir a ºC
  return temp;
};

void TermistorNtc::calcularBeta()
{
  this->B = (log(this->RT1 / this->RT2)) / ((1 / this->T1) - (1 / this->T2));
};

float TermistorNtc::getTemp()
{
  return this->temperatura;
}

void TermistorNtc::update()
{
  float media = 0;
  int i;
  float resultado;
  analogRead(this->Pin);
  delay(10);
  for (i = 0; i < this->NUM_MUESTRAS; i++)
  {
    media += analogRead(this->Pin);
    delay(10);
  }
  media /= this->NUM_MUESTRAS;
  resultado = this->calcularTemp(media);
  this->temperatura = resultado;
};
#endif
