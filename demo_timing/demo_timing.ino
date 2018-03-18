/*
 * Demo Code: Timer Based Interaction
 * -----------------------------------
 * Actuate one channel for x seconds
 * Turn off for x seconds
 * Swap channels
 * Actuate
 * Off
 * ...
 * Tune the times to the needed acuation time for the desired demos.  
 * Color change may take longer. 
 * 
 * This demo includes no bluetooth, captouch, or interaction, only timed actuation 
 */

#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif


/**
 * hardware pins
 */
#define muxApin 6
#define muxBpin 5
#define driveSignalPin 2 // drive circuit signal pin
#define onboardLED 13

#define ONTIME 5000
#define OFFTIME 5000


//so that we don't send "braid touched" while drive circuit is running
bool driving = false; 
int mux_a_channel = 0;

/****************************************
 * End of variable set up
 ****************************************/
 
void setup()
{ 
  Serial.begin(9600);  
  pinMode(muxApin, OUTPUT);
  pinMode(muxBpin, OUTPUT);
  pinMode(driveSignalPin, OUTPUT);
  pinMode(onboardLED, OUTPUT);
  //set mux pins according to braid's channel select parameters
  digitalWrite(muxApin, mux_a_channel);
  digitalWrite(muxBpin, 0);
}

void turnOnDrive() {
   digitalWrite(driveSignalPin, HIGH);
   driving = true;
   Serial.println("Driving...");
}

void turnOffDrive() {
   digitalWrite(driveSignalPin, LOW);
   driving = false;
   Serial.println("Not driving...");
}

void loop()
{
  //switch to other channel
  mux_a_channel = (mux_a_channel + 1) % 2;
  digitalWrite(muxApin, mux_a_channel);
  
  turnOnDrive();
  digitalWrite(13, HIGH);
  delay(ONTIME);
  turnOffDrive();
  digitalWrite(13, LOW);
  delay(OFFTIME);
}


