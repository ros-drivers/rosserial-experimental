#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <WProgram.h>

class Button{
public:

Button(int pin, bool invert  =false , int debounce_delay = 50){
    pin_ = pin;
    debounce_delay= debounce_delay;
    prior_value = digitalRead(pin_);
    last_debounce_time  = millis();
    invert_ = invert;
    changed_= false;
}

bool pressed(){
    return invert_ && digitalRead(pin_);
}

bool changed(){
    bool reading = pressed();
     if (prior_value!= reading){
       prior_value = reading;
       last_debounce_time = millis();
      changed_ = true;
    }
    //if the button value has not changed for the debounce delay, we know its stable
  if (  changed_ &&  ( (millis() - last_debounce_time)  > debouce_delay_ ) ) {
      
      changed_ = false;
    return true;
  }
  return false;
}

private:
int pin_;
bool prior_value;
unsigned long last_debounce_time;
int debouce_delay_;
bool invert_;
bool changed_;

};

#endif
