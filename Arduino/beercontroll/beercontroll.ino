#include "config.h"
#include "termistor.h"
#include "quemador.h"
#include <TimerOne.h>
Quemador QLicor, QMacerador;
TermistorNtc TLicor, TMacerador;
int LicorTO = 0;
long Etapas = 0;

void setup()
{
  QLicor.init(QUEMADOR_LICOR);
  QMacerador.init(QUEMADOR_MACERADOR);
  TLicor.init(SENSOR_LICOR);
  TMacerador.init(SENSOR_MACERADOR);
  // Timer1.initialize(30 * 1000 * 1000);
  //  Timer1.attachInterrupt(ISR_PrimeraEtapa);
  Serial.begin(BAUD_RATE);
}

void enviarSerie()
{
  Serial.print("{");
  Serial.print("\"licorTemp\":");
  Serial.print(int(TLicor.getTemp()));
  Serial.print(" ,");
  Serial.print("\"maceradorTemp\":");
  Serial.print(int(TMacerador.getTemp()));
  Serial.print(" ,");
  Serial.print("\"maceradorQuemador\":");
  Serial.print(QMacerador.isOn());
  Serial.print(" ,");
  Serial.print("\"licorQuemador\": ");
  Serial.print(QLicor.isOn());
  Serial.println("}");
}

void ISR_PrimeraEtapa()
{
}
void loop()
{
  TLicor.update();
  TMacerador.update();
  if (Serial.available() > 0)
  {
    int d =  Serial.parseInt();
    if (d > 0)
    {
      LicorTO = d;
    }
    Serial.println("RECIVIDO: " + String(d));
  }
  if (TLicor.getTemp() < LicorTO)
  {
    QLicor.prender();
  }
  else
  {
    QLicor.apagar();
  }
  enviarSerie();
  delay(TIEMPO_MUESTREO);
}
