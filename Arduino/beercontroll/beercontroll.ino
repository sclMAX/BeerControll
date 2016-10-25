#include "config.h"
#include <TimerOne.h>
#include <U8glib.h>
#include "termistor.h"
#include "quemador.h"

#define KILL_PIN 41 //[RAMPS14-SMART-ADAPTER]
#define LCD_PINS_RS 16
#define LCD_PINS_ENABLE 17
#define LCD_PINS_D4 23
#define LCD_PINS_D5 25
#define LCD_PINS_D6 27
#define LCD_PINS_D7 29

#define BEEPER 37

#define BTN_EN1 31
#define BTN_EN2 33
#define BTN_ENC 2
//35

U8GLIB_ST7920_128X64_1X u8g(LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS);

Quemador QLicor, QMacerador;
TermistorNtc TLicor, TMacerador;
int LicorTO = 15;
long Etapas = 0;

//prueba encoder
int n = LOW;
//fin
void setup()
{
  QLicor.init(QUEMADOR_LICOR);
  QMacerador.init(QUEMADOR_MACERADOR);
  TLicor.init(SENSOR_LICOR);
  TMacerador.init(SENSOR_MACERADOR);
  // Timer1.initialize(30 * 1000 * 1000);
  //  Timer1.attachInterrupt(ISR_PrimeraEtapa);
  pinMode(KILL_PIN, INPUT);
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
  ///PRUEBAS******************
  n = digitalRead(KILL_PIN);
  if (n > 0 )
  {
    digitalWrite(BEEPER, HIGH);
  }
  else
  {
    digitalWrite(BEEPER, LOW);
  }
  Serial.println("n:" + String(n));

  //*****************************

  TLicor.update();
  TMacerador.update();
  if (TLicor.getTemp() < LicorTO)
  {
    QLicor.prender();
  //  digitalWrite(BEEPER, HIGH);
  }
  else
  {
    QLicor.apagar();
  }
  u8g.firstPage();
  do
  {
    draw();
  } while (u8g.nextPage());
 // enviarSerie();
  delay(TIEMPO_MUESTREO);
}

void draw(void)
{
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_unifont);
  int tl = TLicor.getTemp();
  String r = "TL:" + String(tl) + " C";
  u8g.drawStr(0, 10, r.c_str());
  tl = TMacerador.getTemp();
  r = "TM:" + String(tl) + " C";
  u8g.drawStr(0, 25, r.c_str());
  u8g.drawStr(0, 40, "BC v0.1 by MAX");
}
