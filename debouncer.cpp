
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
 * For more info, see the associated wiki in github.
 *===========================================================================*/   
#include <Arduino.h>
#include "debouncer.h"

#define DEFAULT_DEBOUNCE_TIME 50  // in ms;

// to avoid rollover and to conserve any LCD space, this is the maximum 
// number of bounces we'll count
#define MAX_BOUNCES 999 

/*========================================================================
 * INTERNAL METHOD: _init
 *
 * Called by the constructor to initialize the internals of the 
 * debouncer object.
 *
 * Parameters:  
 *   pin - which Arduino pin to debounce.  This pin needs to be set up
 *         externally (with pinMode).  It needs to be a digital pin.
 *
 *   debounceTime - debounce time, in MS
 *======================================================================*/
void Debouncer::_init(int pin, int debounceTime)
{
  int currentPinVal;
  
  _pin = pin;
  _debounceTime = debounceTime;
  _consecutiveReads = 0;
  _debug = false;
  _numBounces = 0;

  currentPinVal = digitalRead(_pin);
  
  if (currentPinVal == HIGH)
  {
    _state = PIN_HIGH;
  }
  else
  {
    _state = PIN_LOW;
  }

}  // end of _init

/*=========================================================================
 * CONSTRUCTOR:  Debouncer(pin)
 *
 * This function sets up the debouncer object for the specified pin, 
 * using the default debounce time.
 *=======================================================================*/
Debouncer::Debouncer(int pin)
{
  _init(pin, DEFAULT_DEBOUNCE_TIME);
}

/*=========================================================================
 * CONSTRUCTOR:  Debouncer(pin, debounceTime)
 *
 * This function sets up the debouncer object for the specified pin, 
 * using the specified debounce time.
 *=======================================================================*/
Debouncer::Debouncer(int pin, int debounceTime)
{
  _init(pin, debounceTime);
}

/*=========================================================================
 * PUBLIC METHOD:  debug(on)
 *
 * This function either turns on or off debugging.  Calling with true will
 * turn debugging on, false will turn it off.  Note debugging is off by 
 * default.
 *=======================================================================*/
void Debouncer::debug(bool on)
{
  _debug = on;
}

/*=========================================================================
 * PUBLIC METHOD: read 
 *
 * This function returns the debounced value of the pin in question.
 * Note that this may or may not reflect the instantaneous value of the pin.
 *=======================================================================*/
int Debouncer::read( void )
{
  unsigned long currentTime;
  int currentRead;
  int filteredRead;

  currentRead = digitalRead(_pin);

  switch (_state)
  {
    case PIN_HIGH:
      if (currentRead == LOW)
      {
        // this is a transition from high to low. we want to start marking time.
        // when the pin is low for long enough, then we'll actually 
	// return a LOW.
        _transitionStart = millis();
        _state = SWITCHING_LOW;
        _consecutiveReads = 1;

        if (_debug)
        {
          Serial.print("DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" HIGH->LOW: ");
          Serial.println(_transitionStart);
        }
      }

      // irrespective of whether the pin is *currently* HIGH or LOW, we still 
      // want to return it in the HIGH state. 
      filteredRead = HIGH;
    break;

    case PIN_LOW:
      // If our pin was currently low, and we read HIGH, start marking time.
      // when the pin is HIGH for long enough, then we'll actually return HIGH
      if (currentRead == HIGH)
      {
        _transitionStart = millis();
        _state = SWITCHING_HIGH;
        _consecutiveReads = 1;

        if (_debug)
        {
          Serial.print("DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" LOW->HIGH: ");
          Serial.println(_transitionStart);
        }
      }

      // irrespective of whether the pin is *currently* high or low, we 
      // still want to return it in the LOW state.
      filteredRead = LOW;
    break;

    case SWITCHING_LOW:
      // Okay, so we've previously detected a transition from HIGH to LOW.  
      // We want to make sure the pin stays in the low state for long 
      // enough to ACTUALLY return a LOW.  
      if (currentRead == LOW)
      {
        // So far, so good.  Last read was LOW, this one is too.  
	// Have we been LOW for long enough?
        currentTime = millis();
        _consecutiveReads++;
        if (currentTime > _transitionStart + _debounceTime)
        {
          filteredRead = LOW;
          _state = PIN_LOW;

          if (_debug)
          {
            Serial.print("DEBOUNCED PIN ");
            Serial.print(_pin);
            Serial.print(" to LOW.  Reads: ");
            Serial.println(_consecutiveReads);
          }
        }
        else
        { 
          // stay in this state a while longer...
          filteredRead = HIGH;
        }
      }
      else
      {
        // so we were going to LOW, but then the pin bounced back to HIGH 
	// before we stayed long enough.  
        // Hello, bounce.  Go back to the straight "HIGH" state.
        if (_debug)
        {
          Serial.print("*** DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" bounced LOW->HIGH.  Reads: ");
          Serial.print(_consecutiveReads);
          Serial.println("  Staying HIGH");
        }

	// Count the number of times we've bounced for debugging purposes
	// Cap out at MAX_BOUNCES to avoid rollover.
	if (_numBounces < MAX_BOUNCES)
	{
	  _numBounces++;
	}

        filteredRead = HIGH;
        _state = PIN_HIGH;
      }
    break;

    case SWITCHING_HIGH:
      // Okay, so we've previously detected a transition from low to high.  
      // We want to make sure the pin stays in the high state for long 
      // enough to ACTUALLY return HIGH.  
      if (currentRead == HIGH)
      {
        // So far, so good.  Last read was high, this one is too.  
	// Have we been HIGH for long enough?
        //              ^^^^  (insert Grateful Dead joke here)
        currentTime = millis();
        _consecutiveReads++;
        if (currentTime > _transitionStart + _debounceTime)
        {
          filteredRead = HIGH;
          _state = PIN_HIGH;
          
          if (_debug)
          {
            Serial.print("DEBOUNCED PIN ");
            Serial.print(_pin);
            Serial.print(" to HIGH.  Reads: ");
            Serial.println(_consecutiveReads);
          }
        }
        else
        { 
          // stay in this state a while longer...
          filteredRead = LOW;
        }
      }
      else
      {
        // so we were going to HIGH, but then the pin switched back to low 
	// before we stayed long enough.  
        // Hello, bounce.  Go back to the straight "LOW" state.
        if (_debug)
        {
          Serial.print("*** DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" bounced HIGH->LOW.  Reads: ");
          Serial.print(_consecutiveReads);
          Serial.println("  Staying LOW");
        }

	// Count the number of times we've bounced for debugging purposes
	// Cap out at MAX_BOUNCES to avoid rollover.
	if (_numBounces < MAX_BOUNCES)
	{
	  _numBounces++;
	}

        filteredRead = 0;
        _state = PIN_LOW;
      }
    break;
  }

  return filteredRead;
  
}  // end of read

/*=========================================================================
 * PUBLIC METHOD:  getNumBounces 
 *
 * This function returns the number of times our debouncer has actually
 * "bounced" per power up.  We cap out at MAX_BOUNCES.
 *=======================================================================*/
int Debouncer::getNumBounces( void )
{
  return _numBounces;
}
