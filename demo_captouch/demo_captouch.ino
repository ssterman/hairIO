/**
 * Demo code for bluetooth enabled PCB
 * ***********************************
 * Uses Bluefruit app on phone to receive captouch signals and toggle driving
 * Handles two channels of input and output
 * Monitors temperature via thermistor; includes drive shutoff if temperature threshold exceeded
 * Monitors battery voltages; warning light and drive shutoff if voltages below threshold
 */

//****************************************************************************************
// Adapted from https://github.com/Illutron/AdvancedTouchSensing
// Moves the gesture analysis from Processing to run entirely on Arduino, with fixed thresholds
// July 21 2017
//
//Added Bluetooth code adapted from Adafruit nRF51822 Bluefruit  source code
//****************************************************************************************
//****************************************************************************************
// Illutron take on Disney style capacitive touch sensor using only passives and Arduino
// Dzl 2012
//****************************************************************************************


//                              10n
// PIN 9 --[10k]-+-----10mH---+--||-- OBJECT
//               |            |
//              3.3k          |
//               |            V 1N4148 diode
//              GND           |
//                            |
//Analog 0 ---+------+--------+
//            |      |
//          100pf   1MOmhm
//            |      |
//           GND    GND

#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif


/**
 * capacitive touch definitions
 */
#define SET(x,y) (x |=(1<<y))        //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))           // |
#define CHK(x,y) (x & (1<<y))               // |
#define TOG(x,y) (x^=(1<<y))                //-+
#define N 160  //How many frequencies

/**
 * hardware pins
 */
#define muxApin 6
#define muxBpin 5
#define CAPTOUCHPIN A3
#define onboardLED 13

/**
 * pins specific to a particular braid
 * are stored in a braidx object
 * the channel on the mux is set by the state of the
 * A and B select channels; the relevant states are saved 
 * in the braid object.
 * ch0 = A0B0
 * ch1 = A1B0
 */
struct braidx {
  int muxSelectA;
  int muxSelectB;
  int thermPin;
  String name_message;
  float gesturePoints[2][2];
  int eeprom_start_addr;
};

struct index_val {
  int index;
  int val;
};

/**
 * hardware has two braid channels
 */ 
struct braidx braid0 = {0, 0, A0, "Braid 0", { {0,0}, {0,0} }, 0}; //left
struct braidx braid1 = {1, 0, A1, "Braid 1", { {0,0}, {0,0} }, 8}; //right
struct braidx currentBraid = braid0;

/**
 * Gesture sensing and processing variables
 */
//Gesture sensing variables
int results[N];            //-Filtered result buffer
int freq[N];            //-Filtered result buffer
int sizeOfArray = N;

//Gesture processing variables
int gestureDist[2];
String names[2] = {"nothing", "touch"};

char curGesture = 0;
char lastGesture = 0;

/****************************************
 * End of variable set up
 ****************************************/

void setup()
{ 
  Serial.begin(9600);
  Serial.println("demo_captouch edited March 18, 2018");
  
  TCCR1A = 0b10000010;      //-Set up frequency generator
  TCCR1B = 0b00011001;      //-+
  ICR1 = 110;
  OCR1A = 55;

  pinMode(9, OUTPUT);       //-Signal generator pin
  pinMode(8, OUTPUT);       //-Sync (test) pin
  pinMode(muxApin, OUTPUT);
  pinMode(muxBpin, OUTPUT);
  pinMode(onboardLED, OUTPUT);

  // initialize results array to all zeros
  memset(results,0,sizeof(results));

  //set mux pins according to braid's channel select parameters
  digitalWrite(muxApin, currentBraid.muxSelectA);
  digitalWrite(muxBpin, currentBraid.muxSelectB);

  setGestureThresholds(&braid0);
  setGestureThresholds(&braid1);
}

void setGestureThresholds(struct braidx * braid) {
  //no touch
  braid->gesturePoints[0][0] = 34;
  braid->gesturePoints[0][1] = 390;

  //touch 
  braid->gesturePoints[1][0] = 50;
  braid->gesturePoints[1][1] = 300;
}


void processGesture() {
  if (curGesture == 1){
    String s = currentBraid.name_message + " touched";
    Serial.println(s);
  }
}

//adapted from http://forum.arduino.cc/index.php?topic=41999.0
struct index_val getMaxFromArray(int* array, int size) {
  int max = array[0];
  int max_index = 0;
  for (int i = 1; i < size; i++) {
    if (max < array[i]) {
      max = array[i];
      max_index = i;
    }
  }
  struct index_val iv = {max_index, max};
  return iv;
}

//assumes no negative values for time or voltage
int dist(int x1, int y1, int x2, int y2) {

  int xmax = max(x1, x2);
  int ymax = max(y1, y2);

  float w;
  if (x1 > x2) {
    w = x1 - x2;
  } else {
    w = x2 - x1;
  }
  float h;
  if (y1 > y2) {
    h = y1 - y2;
  } else {
    h = y2 - y1;
  }
  
  int intVal = (int) sqrt(h*h + w*w);
  return intVal;
}

void analyzeInput(int timeArr[], int voltageArr[]) {

  /* ====================================================================
    Gesture compare
    ====================================================================  */
//  int currentMax = 0;
//  int currentMaxValue = -1;
//  for (int i = 0; i < 2; i++)
//  {
//    //calculate individual dist
//    struct index_val iv = getMaxFromArray(voltageArr, N);
//    gestureDist[i] = dist(getMaxFromArray(timeArr, N).val, getMaxFromArray(voltageArr, N).val, currentBraid.gesturePoints[i][0], currentBraid.gesturePoints[i][1]);
//    if (gestureDist[i] < currentMaxValue || i == 0)
//    {
//      currentMax = i;
//      currentMaxValue =  gestureDist[i];
//    }
//  }

  int max_ = getMaxFromArray(voltageArr, N).val;
  float touch_dist = abs(max_ - currentBraid.gesturePoints[1][1]);
  float notouch_dist = abs(max_ - currentBraid.gesturePoints[0][1]);
  if (touch_dist < notouch_dist) {
    curGesture = 1;
  } else {
    curGesture = 0;
  }
 
//  int type = currentMax;
//  lastGesture = curGesture;
//  curGesture = type;
}


void capacitiveSweep() {
  for (unsigned int d = 0; d < N; d++)
  {
    int v = analogRead(CAPTOUCHPIN);  //-Read response signal
    CLR(TCCR1B, 0);         //-Stop generator
    TCNT1 = 0;              //-Reload new frequency
    ICR1 = d;               // |
    OCR1A = d / 2;          //-+
    SET(TCCR1B, 0);         //-Restart generator

    int testVal = results[d] * 0.5 + v*0.5;
    results[d] = testVal;
    //results[d] = results[d] * 0.5 + (float)(v) * 0.5; //Filter results
    freq[d] = d;

    //   plot(v,0);              //-Display
    //   plot(results[d],1);
    // delayMicroseconds(1);
  }
}


void do_captouch(){
   //update the captouch state
  capacitiveSweep();
  
  //this would send the data to processing
  //PlottArray(1,freq,results);

  // instead of sending to processing, do the work here
  analyzeInput(freq, results);
  processGesture();
}


void reset_mux(struct braidx newbraid){
  currentBraid = newbraid;
  digitalWrite(muxApin, currentBraid.muxSelectA);
  digitalWrite(muxBpin, currentBraid.muxSelectB);
}

void loop()
{
  reset_mux(braid0);
  do_captouch();

  reset_mux(braid1);
  do_captouch();

  TOG(PORTB, 0);           //-Toggle pin 8 after each sweep (good for scope)
}


