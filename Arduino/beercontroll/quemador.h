#ifndef QUEMADOR_H
#define QUEMADOR_H
class Quemador
{
  public:
    Quemador();
    void init(int); //Inicializa el quemador en el pin indicado
    bool isOn();
    void prender();
    void apagar();

  private:
    int Pin;
    bool on = false;
    void setPin(int);
    void setOn();
    void setOff();
};

Quemador::Quemador(){

};

void Quemador::init(int pin)
{
    this->setPin(pin);
    pinMode(this->Pin, OUTPUT);
}

bool Quemador::isOn()
{
    return this->on;
}

void Quemador::prender()
{
    digitalWrite(this->Pin, HIGH);
    this->setOn();
}

void Quemador::apagar()
{
    digitalWrite(this->Pin, LOW);
    this->setOff();
}

void Quemador::setPin(int pin)
{
    this->Pin = pin;
}

void Quemador::setOn()
{
    this->on = true;
}

void Quemador::setOff()
{
    this->on = false;
}

#endif
