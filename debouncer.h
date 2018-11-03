#ifndef DEBOUNCER_H
#define DEBOUNCER_H
/*============================================================================
 * DEBOUNCER LIBRARY
 *
 * This library provides a simple method to debounce a pin in software.
 *
 * USAGE: 
 *   First, create debouncer object for the pin in question.  You can optionally
 *   specify a debounce time...if none is provided, 50 ms is used.
 *
 *   Instead of doing digitalReads from the pin, you can then use the "read" 
 *   method in order to read the pin in question.
 *
 * NOTES:
 *   This library uses millis() for timing.
 *   It assumes that reads happen much faster than the debounce time.
 *   the "debug" method will turn on and off debug information via the 
 *   serial port.  It assumes that you have initialized that port seperately.
 *
 * For more information, see the associated wiki in github.
 *===========================================================================*/   
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
    int getNumBounces( void );
  private:
    bool               _debug;
    _debounceStateType _state;  
    int                _debounceTime;
    unsigned long      _transitionStart;
    int                _consecutiveReads;
    int                _pin;
    void               _init(int pin, int debounceTime);
    int                _numBounces;
};

#endif // DEBOUNCER_H
