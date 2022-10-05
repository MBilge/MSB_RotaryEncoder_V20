/*
 * MSB_RotaryEncoder.cpp
 * Dynamic speed full step Rotary Encoder library for Arduino
 * 
 * Modified by Mehmet S. Bilge October 2021    
 * Version 1.0
 * Unfortunelately switch does not stable. gives inital unwanted reading, couldnt find the reason
 * and reading switch is slow     
 */


#include "MSB_RotaryEncoder_V20.h"

RotaryEncoder::RotaryEncoder(int PinCLK, int PinDT, int PinSW)
{
  //Definitions
  pinA  = PinCLK;
  pinB  = PinDT;
  pinSW = PinSW;
  
  enableInternalRotaryPullups(DEFAULT_rotaryPullUp);
  setValue(minValue);
  
  //pinMode(pinSW, INPUT_PULLUP);
  
#ifdef ARDUINO_ARCH_STM32
  pinMode(pinSW, (WiringPinMode)INPUT_PULLUP);
#else
  pinMode(pinSW, INPUT_PULLUP);
#endif


  _lastTransition = millis(); // current time
  _switchState = StateIdle;         // start with idle
  _new = false;               // new reading from switch
  
}

// ----- Rotary operations -----

void RotaryEncoder::setRotaryLimits(int newMin, int newMax) 
{
  minValue = newMin;
  maxValue = newMax;
}

void RotaryEncoder::setWrapMode(bool newWrapMode)
{
  wrapMode = newWrapMode;  
}

void RotaryEncoder::setRotaryLogic(bool logic) 
{
  rotaryLogic = (logic);
}

void RotaryEncoder::enableInternalRotaryPullups(bool PullUp)
{
    if (PullUp)
      {
          pinMode(pinA , INPUT_PULLUP);
          pinMode(pinB , INPUT_PULLUP);
      }
      else 
      {
          pinMode(pinA , INPUT);
          pinMode(pinB , INPUT);      
      }
}

// Retrieves the rotation direction 
// CCW = -1
// NOT_MOVED = 0
// CW = 1
int RotaryEncoder::getDirection()
{
    return direction;
}

// Returns the current absolute position of the rotary encoder
// betwen minLimit and maxLimit
int RotaryEncoder::getValue()
{
    return value;
}

// Returns the current incremental rotary state
// -3: Counter clockwise turn, multiple notches fast
// -2: Counter clockwise turn, multiple notches
// -1: Counter clockwise turn, single notch
//  0: No change
//  1: Clockwise turn, single notch
//  2: Clockwise turn, multiple notches
//  3: Clockwise turn, multiple notches fast
int RotaryEncoder::getState()
{
    return state;
}

// Sets a starting position
void RotaryEncoder::setValue(int newValue)
{
    if ((newValue >= minValue ) && (newValue <= maxValue)) value = newValue;
  else value = 0;
  direction = NOT_MOVED;
}

 // Sets the maximum value of the rotation counter
void RotaryEncoder::setMaxValue(int newMaxValue)
{
    maxValue = newMaxValue;
}

// Sets the minimum value of the rotation counter
void RotaryEncoder::setMinValue(int newMinValue)
{
    minValue = newMinValue;
  setValue(minValue);
}

// Sets the sensitivity of rotation between 0 and 255
void RotaryEncoder::setSensitivity(byte speed)
{
    if ( (speed >=0) && (speed <=255)) sensitivity = speed;
  else sensitivity = 0;
}

// Sets the step that position changes in every transition
void RotaryEncoder::setRotationalStep(byte step)
{
    rotationalStep = step;
}

// Updates only the encoder state.
void RotaryEncoder::rotaryUpdate()
{
  int pinState; 
  unsigned long timeStamp;
  unsigned long changeTime;
   // Sample rotary digital pins and swap them if neededd
  if (rotaryLogic) pinState = (digitalRead(pinA) << 1) | digitalRead(pinB);
  else             pinState = (digitalRead(pinB) << 1) | digitalRead(pinA);
  
  // Serial.print("pinState : "); Serial.print(pinState);
  // Determine new state from the pins and state table.
  _state  = pgm_read_byte(&fullStepTable[_state & 0x0f][pinState]);
  // Serial.print("  _state : "); Serial.println(_state);
  
  // Check rotary state and direction
  switch (_state & 0x30) {
      case DIR_CW:
            state = 1;
            direction = CW;
            break;
      case DIR_CCW:
            state = -1;
            direction = CCW;
            break;
      case DIR_NONE:
            state = 0;
            direction = NOT_MOVED;
            break;
  }
  
  // Check if rotary changed
  if (state != 0) 
  {
      // calculate the time difference between the states
      timeStamp  = millis();
      changeTime = timeStamp - lastTime;
      lastTime   = timeStamp;
      // Adjust the state according to sensitivity
      if (changeTime < (sensitivity / 2)) {
          state *= 3;
      } else if (changeTime < sensitivity) {
          state *= 2;
      }   
  } 
  // Updating the Rotary Value
  value +=  state * rotationalStep;
  //Serial.print(" value in lib : "); Serial.println(value);
  if (wrapMode) {
    if      (value > maxValue) value = minValue;
    else if (value < minValue) value = maxValue;
  } 
  else 
  {
    if      (value > maxValue) value = maxValue;
    else if (value < minValue) value = minValue;
  }
}

// ----- Switch operations -----


// Sets the switch debouncing time in milliseconds
void RotaryEncoder::setSwitchDebounceDelay(unsigned long _DEBOUNCE_DELAY) 
{
  DEBOUNCE_DELAY = _DEBOUNCE_DELAY;
}

// Sets the long click time 
void RotaryEncoder::setLongClickDelay(unsigned long _LONGCLICK_DELAY)
{
  LONGCLICK_DELAY = _LONGCLICK_DELAY; 
}

// Sets the mutiple click interval time in milliseconds
void RotaryEncoder::setSingleClickDelay(unsigned int _SINGLECLICK_DELAY)
{
  SINGLECLICK_DELAY = _SINGLECLICK_DELAY;
}


// Updates only the switch state.
void RotaryEncoder::switchUpdate()
{
  _new = false;  
  
  bool pressed = (digitalRead(pinSW) == _pinActiveLevel ); // read the switch
  
  if (!pressed && _switchState == StateIdle) { return; } // if no click then go back

  unsigned int now = millis();       // get the current time
  int diff = now - _lastTransition;  // get the differantial

  State next = StateIdle;  // set the next state as idle
  switch (_switchState) {
    case StateIdle:                next = _checkIdle(pressed, diff);                break;
    case StateDebounce:            next = _checkDebounce(pressed, diff);            break;
    case StatePressed:             next = _checkPressed(pressed, diff);             break;
    case StateClickUp:             next = _checkClickUp(pressed, diff);             break;
    case StateClickIdle:           next = _checkClickIdle(pressed, diff);           break;
    case StateSingleClick:         next = _checkSingleClick(pressed,diff);          break;
    case StateDoubleClickDebounce: next = _checkDoubleClickDebounce(pressed, diff); break;
    case StateDoubleClick:         next = _checkDoubleClick(pressed, diff);         break;
    case StateLongClick:           next = _checkLongClick(pressed, diff);           break;
    case StateOtherUp:             next = _checkOtherUp(pressed,diff);              break;
    }
  
  // if this is new a click
  if (next != _switchState) {
    _lastTransition = now; // Mark current state transition to make 'diff' meaningful
    _switchState = next; // Enter next state
    // Only mark this state 'new' for one iteration, to cause
    // e.g. isClick() to return true until the next call to update()
    _new = true; 
    }

}   

bool RotaryEncoder::isClick() const {
  return _new && (_switchState == StatePressed || _switchState == StateDoubleClick);
}

/**
 * True when a Single click is detected, i.e. it will not trigger before
 * e.g. a Double click.
 */
bool RotaryEncoder::isSingleClick() {
  return _new && _switchState == StateSingleClick;
}

/**
 * True when a Double click is detected.
 */
bool RotaryEncoder::isDoubleClick() {
  return _new && _switchState == StateDoubleClick;
}

/**
 * True when a Long click is detected.
 */
bool RotaryEncoder::isLongClick() {
  return _new && _switchState == StateLongClick;
}

/**
 * True once the button is released after Click, Long click or Double click.
 *
 * Note: there is no release event after a Single click, because that is a
 * 'synthetic' event that happens after a normal click.
 */
bool RotaryEncoder::isReleased() {
  return _new && (_switchState == StateClickUp || _switchState == StateOtherUp);
}

State RotaryEncoder::_checkIdle(bool pressed, int diff) {
  (void)diff;
  // Wait for a key press
  return pressed ? StateDebounce : StateIdle;
}

State RotaryEncoder::_checkDebounce(bool pressed, int diff) {
  // If released in this state: it was a glitch
  if (!pressed) { return StateIdle; }
    // Still pressed after debounce delay: real 'press'  
  if (diff >= DEBOUNCE_DELAY) { return StatePressed; }
  return StateDebounce;
}

State RotaryEncoder::_checkPressed(bool pressed, int diff) {
  // If released, go wait for either a double-click, or
  // to generate the actual SingleClick event,
  // but first mark that the button is released.
  if (!pressed) {
    return StateClickUp;
  }
  // If pressed, keep waiting to see if it's a long click
  if (diff >= LONGCLICK_DELAY) {
    return StateLongClick;
  }
  return StatePressed;
}

State RotaryEncoder::_checkClickIdle(bool pressed, int diff) {
  if (pressed) {
    return StateDoubleClickDebounce;
  }
  if (diff >= SINGLECLICK_DELAY) {
    return StateSingleClick;
  }
  return StateClickIdle;
}

State RotaryEncoder::_checkClickUp(bool pressed, int diff) {
  (void)pressed;
  (void)diff;
  return StateClickIdle;
}

State RotaryEncoder::_checkSingleClick(bool pressed, int diff) {
  (void)pressed;
  (void)diff;
  return StateIdle;
}


State RotaryEncoder::_checkDoubleClickDebounce(bool pressed, int diff) {
  if (!pressed) { return StateClickIdle; }
  if (diff >= DEBOUNCE_DELAY) { return StateDoubleClick; }
  return StateDoubleClickDebounce;
}

State RotaryEncoder::_checkDoubleClick(bool pressed, int diff) {
  (void)diff;
  if (!pressed) { return StateOtherUp; }
  return StateDoubleClick;
}

State RotaryEncoder::_checkLongClick(bool pressed, int diff) {
  (void)diff;
  if (!pressed) { return StateOtherUp; }
  return StateLongClick;
}

State RotaryEncoder::_checkOtherUp(bool pressed, int diff) {
  (void)pressed;
  (void)diff;
  return StateIdle;
}



// Updates both the rotary and switch.
void RotaryEncoder::update()
{
  rotaryUpdate();
  switchUpdate();
}

