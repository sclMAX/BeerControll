//Encoder rotativo.
//Hacia la derecha aumenta en una unidad y hacia la izquierda a la inversa.
#include "pins_RAMPS.h"
int encoderPin1 = BTN_EN1;
int encoderPin2 = BTN_EN2;
int encoderSwitchPin = BTN_ENC; //Pulsador

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

int valor;

void setup() 
{
  Serial.begin (9600);  //Se puede cambiar a otra velocidad de transmisión sin problemas. 

  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
  pinMode(encoderSwitchPin, INPUT);

  digitalWrite(encoderPin1, HIGH); //Pullup resistor ON
  digitalWrite(encoderPin2, HIGH); //Pullup resistor ON
  digitalWrite(encoderSwitchPin, HIGH); //Pullup resistor ON
}

void loop()
{ 
  updateEncoder();
  if(digitalRead(encoderSwitchPin)==LOW)
  {
    encoderValue=0;  //Reinicia el contador de pulsos con el pulsador.
  }
  else
  {
    valor=(encoderValue/4);  //Permite ajustar cada posición del encoder rotativo con un valor.
  } 
  Serial.println(valor);
  delay(100); 
}


void updateEncoder()
{
  int MSB = digitalRead(encoderPin1); //MSB = bit mas significativo
  int LSB = digitalRead(encoderPin2); //LSB = bit menos significativo

  int encoded = (MSB << 1) |LSB; //convierte el pin 2 en un solo valor
  int sum  = (lastEncoded << 2) | encoded; //añadiendolo al valor anterior

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //guarda el valor para la proxima vez
}





