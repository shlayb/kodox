#include <Arduino.h>

class Motor{
    public:
    Motor(int pin);
    int pin;
    void init();
    void write(int value);
}

Motor::Motor(int pin){
    this->pin = pin;
}

Motor::init(){
    pinMode(pin, OUTPUT);
    LedcaAttach(pin, 5000, 8);
}

Motor::write(int value){
    ledcWrite(pin, value);
}