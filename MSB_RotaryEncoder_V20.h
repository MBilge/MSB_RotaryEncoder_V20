/*
 * MSB_RotaryEncoder.h
 * Dynamic speed full step Rotary Encoder library for Arduino
 * 
 * Modified by Mehmet S. Bilge October 2021  
 *  
 * Version 2.0     
 */

#pragma once

#ifndef MSB_RotaryEncoder_H
#define MSB_RotaryEncoder_H

// #if ARDUINO >= 100
#include <Arduino.h>
// #else
//   #include <WProgram.h> 
// #endif

/* ----  DEFAULT CONFIGURATION ---- */

// Default min and max rotational limits. 
// Can be also set with setRotaryLimits() 
#define DEFAULT_minValue  -10 // minlimit for Rotary value
#define DEFAULT_maxValue   10 // maxlimit for Rotary value

// Default wrap-around mode for value  true/false
#define DEFAULT_wrapMode false  // Do not wrap-around

// Set rotational direction (inverts the CLK and DT pins) true/false 
#define DEFAULT_rotationalLogic true  // clockwise (left to right turning)

// Sets the sensitivity of rotation
// Sensitivity value 0..255 
// the higher the sensitivity the faster increasing of value 
#define DEFAULT_sensitivity 0

// Set default rotational step  
#define DEFAULT_rotationalStep 1

// Set Rotary pin pull up
#define DEFAULT_rotaryPullUp false

#define DIR_NONE  0x00      // No Move
#define DIR_CW    0x10      // Clockwise 
#define DIR_CCW   0x20      // Counter-clockwise 

// Use the full-step state table, clockwise and counter clockwise
#define RFS_START          0x00     //!< Rotary full step start
#define RFS_CW_FINAL       0x01     //!< Rotary full step clock wise final
#define RFS_CW_BEGIN       0x02     //!< Rotary full step clock begin
#define RFS_CW_NEXT        0x03     //!< Rotary full step clock next
#define RFS_CCW_BEGIN      0x04     //!< Rotary full step counter clockwise begin
#define RFS_CCW_FINAL      0x05     //!< Rotary full step counter clockwise final
#define RFS_CCW_NEXT       0x06     //!< Rotary full step counter clockwise next

static const PROGMEM uint8_t fullStepTable[7][4] = {
			    
/* RFS_START */     {RFS_START,      RFS_CW_BEGIN,  RFS_CCW_BEGIN, RFS_START},   
/* RFS_CW_FINAL*/   {RFS_CW_NEXT,    RFS_START,     RFS_CW_FINAL,  RFS_START | DIR_CW},
/* RFS_CW_BEGIN*/	  {RFS_CW_NEXT,    RFS_CW_BEGIN,  RFS_START,     RFS_START},
/* RFS_CW_NEXT	*/	{RFS_CW_NEXT,    RFS_CW_BEGIN,  RFS_CW_FINAL,  RFS_START},
/* RFS_CCW_BEGIN */	{RFS_CCW_NEXT,   RFS_START,     RFS_CCW_BEGIN, RFS_START},		    
/* RFS_CCW_FINAL*/	{RFS_CCW_NEXT,   RFS_CCW_FINAL, RFS_START,     RFS_START | DIR_CCW},
/* RFS_CCW_NEXT*/	  {RFS_CCW_NEXT,   RFS_CCW_FINAL, RFS_CCW_BEGIN, RFS_START},
};

//==========================================================================

 #define DEFAULT_DEBOUNCE_DELAY    20  // ms
 #define DEFAULT_SINGLECLICK_DELAY 250 // ms
 #define DEFAULT_LONGCLICK_DELAY   300 // ms



enum State {
    StateIdle,         // idle no click
    StateDebounce,
    StatePressed,
    StateClickUp,
    StateClickIdle,
    StateSingleClick,  // real single click
    StateDoubleClickDebounce,
    StateDoubleClick,  // real double click
    StateLongClick,    // real long click
    StateOtherUp,
};

class RotaryEncoder
{
public:
	// Constructor
	RotaryEncoder(int PinCLK, int PinDT, int PinSW);
    
    void update();          // Updates the states of the internal values, both for the rotary and the switch.
    
    // ------ Rotary Functions and definitions -------
    const byte CCW = -1;
    const byte NOT_MOVED = 0;
    const byte CW = 1;     

    void rotaryUpdate();    // Updates only the encoder state.
    void setRotaryLimits(int newMin, int newMax);    // Sets the limits of the rotary encoder
    void setMaxValue(int newMaxValue);    // Sets the maximum value of the rotation counter
    void setMinValue(int newMinValue);	  // Sets the minimum value of the rotation counter	
    void setWrapMode(bool newWrapMode);   // Sets the wrap mode
    void setRotaryLogic(bool logic);      // Sets the rotary switch logic. Used to invert the rotation direction
    void enableInternalRotaryPullups(bool PullUp);   // Enable internal pullups resistors for the rotary
    int  getDirection();	// Retrieves the rotation direction 
    int  getState();	    // Returns the current incremental rotary state  (-3..0..3)
    int  getValue();	    // Returns the current absolute position of the rotary encoder
    void setValue(int newValue);	    // Sets a starting position
    void setSensitivity(byte fast);     // Sets the sensitivity of rotation between 0 and 255
    void setRotationalStep(byte step);  // Sets the step that position changes in every transition

    //---------- Switch functions -----------------

    void switchUpdate();    // Updates only the switch state.
    
    void setSwitchDebounceDelay(unsigned long dd);   // Sets the switch debouncing time in milliseconds
    void setSingleClickDelay(unsigned int interval);    // Sets the mutiple click interval time in milliseconds    
    void setLongClickDelay(unsigned long longPress); 

    
    bool isClick() const; 
    bool isSingleClick();
    bool isDoubleClick(); 
    bool isLongClick();
    bool isReleased();

private:
	
      // Rotary	
    int pinA, pinB;  // Pins used for the rotary encoder.   (DT, CLK)  
    uint8_t _state = 0;  // store the determined new state from the pins and state table.
    volatile int direction = NOT_MOVED;  //keep the rotary direction
    volatile int value = 0; // keep the absolute value
    volatile int state = 0; // keep the rotary direction and speed
    bool rotaryLogic    = DEFAULT_rotationalLogic; 	    
    int  maxValue 	    = DEFAULT_maxValue ;
    int  minValue 	    = DEFAULT_minValue ;
    bool wrapMode       = DEFAULT_wrapMode; 
    int  rotationalStep = DEFAULT_rotationalStep; // Position changes by 1. Can be set with setRotationalStep
    bool rotaryPullUp   = DEFAULT_rotaryPullUp;
    byte sensitivity    = DEFAULT_sensitivity; 
    unsigned int lastTime = 0; 

    // Switch
    int pinSW;   // Pin used for the switch
    unsigned int _lastTransition;
    int _pinActiveLevel = LOW;

    State _switchState = StateIdle;
    bool _new = false;

    // Default Configuration for switch 
    int DEBOUNCE_DELAY    = DEFAULT_DEBOUNCE_DELAY;    // ms
    int SINGLECLICK_DELAY = DEFAULT_SINGLECLICK_DELAY; // ms
    int LONGCLICK_DELAY   = DEFAULT_LONGCLICK_DELAY;   // ms

    State _checkIdle(bool pressed, int diff);
    State _checkDebounce(bool pressed, int diff);
    State _checkPressed(bool pressed, int diff);
    State _checkClickIdle(bool pressed, int diff);
    State _checkSingleClick(bool pressed, int diff); 
    State _checkClickUp(bool pressed, int diff);
    State _checkDoubleClickDebounce(bool pressed, int diff);
    State _checkDoubleClick(bool pressed, int diff);
    State _checkLongClick(bool pressed, int diff);
    State _checkOtherUp(bool pressed, int diff);

};

#endif // MSB_RotaryEncoder_H

