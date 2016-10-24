#include "config.h"
#include <U8glib.h>
U8GLIB_ST7920_128X64_1X u8g(LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS);
// Valores de medida
int anterior = 1;
int contador;
char texto[10];
int a, b, c, d;
int vA, vB, vC, vD, vaA, vaB, vaC, vaD;

void setup()
{
  a = 52 ;
  b = 53;
  c = 54;
  d = 55;
  pinMode(a, INPUT);
  pinMode(b, INPUT);
  pinMode(c, INPUT);
  pinMode(d, INPUT);
  // Inicializamos las variables
  anterior = 1;
  contador = 0;
  vaA = -1;
  vaB = -1;
  vaC = -1;
  vaD = -1;
  Serial.begin(BAUD_RATE);
}

void loop()
{
  // Lectura de A, si es 0, volvemos a medir para asegurar el valor
  vA = digitalRead(a);
  vB = digitalRead(b);
  vC = digitalRead(c);
  vD = digitalRead(d);
  if ((vaA != vA) || (vaB != vB) || (vaC != vC) || (vaD != vD)) {
    Serial.println("A:" + String(vA) + "B:" + String(vB) + "C:" + String(vC) + "C:" + String(vD)  );
    vaA = vA;
    vaB = vB;
    vaC = vC;
    vaD = vD;
  }
  u8g.firstPage();
  do
  {
    draw();
  } while (u8g.nextPage());
  //delay(TIEMPO_MUESTREO);
}

void draw(void)
{
  u8g.setFont(u8g_font_unifont);
  u8g.drawStr(0, 21, ("A:" + String(vA)).c_str());
  u8g.drawStr(0, 32, ("B:" + String(vB)).c_str());
  u8g.drawStr(0, 43, ("C:" + String(vC)).c_str());
  u8g.drawStr(0, 54, ("D:" + String(vD)).c_str());
}
