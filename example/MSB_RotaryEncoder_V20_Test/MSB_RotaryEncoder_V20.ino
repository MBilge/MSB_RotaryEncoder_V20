/*  An example of an Rotary encoder with switch button
*   by M.S.B 10/2022
*   Setup :
*   5 pin Rotary encoder connected to Arduino board 
*   CLK --> 7 (can be any GPIO)
*   DT  --> 8 (can be any GPIO)
*   SW  --> 9 (can be any GPIO)
*   VCC --> +5V 
*   GND --> GND
*/

/*  The encoder library support many features like speed, direction, value, sensitivity etc.
*   The value generated by encoder can be limited with MIN and MAX
*   with Wrapping mode the value either go back minimum value when it reached the max or vise versa.
*   The switch button can support Single, Double or Long click
*   debounce, single click delay and long click delay can be redefined. 
*/

#include "MSB_RotaryEncoder_V20.h"

RotaryEncoder encoder(7,8,9);  // Nano
int last = 1;
int pos, state;

void setup() {
  Serial.begin(9600);
  
  encoder.setSensitivity(0);
  encoder.setWrapMode(true);  
  //encoder.setRotationalStep(5);
  encoder.setMaxValue(20);
  encoder.setMinValue(1);  
  encoder.setRotaryLogic(true);  
  encoder.enableInternalRotaryPullups(true);

  Serial.println("Encoder is setup...");

}

void loop() {

  encoder.update();
  pos = encoder.getValue();
  //Serial.print("Value : "); Serial.println(pos);
  if (pos != last )
  {
    Serial.print("Value : "); Serial.println(pos);
    last = pos;    
  }
  
  if ( encoder.isSingleClick() ) Serial.println("Single Clicked...");
  if ( encoder.isDoubleClick() ) Serial.println("Double Clicked...");
  if ( encoder.isLongClick()   ) Serial.println("Long Clicked...");

  // put your main code here, to run repeatedly:
}
