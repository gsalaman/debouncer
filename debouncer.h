#ifndef ARDUINO_H
#define ARDUINO_H

#include <Arduino.h>

typedef enum
{
  PIN_HIGH,
  PIN_LOW, 
  SWITCHING_LOW,
  SWITCHING_HIGH
} _debounceStateType;

class Debouncer
{
  public:
    Debouncer(int pin);
    Debouncer(int pin, int debounceTime);
    void debug(bool on);
    int read( void );
  private:
    bool               _debug;
    _debounceStateType _state;  
    int                _debounceTime;
    unsigned long      _transitionStart;
    int                _consecutiveReads;
    int                _pin;
    void               _init(int pin, int debounceTime);
};

#endif // ARDUINO_H