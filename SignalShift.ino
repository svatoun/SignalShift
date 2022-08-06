
// #define DEBUG_PRINT

#include "Arduino.h"
#include <NmraDcc.h>

const byte ShiftPWM_dataPin = 11;
const byte ShiftPWM_clockPin = 13;

const byte ACK_BUSY_PIN = 10;

// You can choose the latch pin yourself.
const byte ShiftPWM_latchPin=8;

unsigned char maxBrightness = 255;
unsigned char minBrightness = 0;
unsigned char pwmFrequency = 100;

const bool ShiftPWM_balanceLoad = false;
const bool ShiftPWM_invertOutputs = true; 

//#define SHIFTPWM_NOSPI
#define SHIFTPWM_USE_TIMER2

#include <ShiftPWM.h> 


void setupShiftPWM() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);

  pinMode(2, INPUT);

  ShiftPWM.SetAmountOfRegisters(4);
  ShiftPWM.SetPinGrouping(1); //This is the default, but I added here to demonstrate how to use the funtion
  ShiftPWM.Start(pwmFrequency,maxBrightness);
  ShiftPWM.SetAll(0);
}

void loopPWM() {
  // put your main code here, to run repeatedly:
  /*
  ShiftPWM.SetAll(0);
  delay(500);
  ShiftPWM.SetAll(maxBrightness);
  delay(500);
  */

  ShiftPWM.PrintInterruptLoad();
  // Fade in and fade out all outputs one by one fast. Usefull for testing your hardware. Use OneByOneSlow when this is going to fast.
  ShiftPWM.OneByOneFast();

  // Fade in all outputs
  for(int j=0;j<maxBrightness;j++){
    ShiftPWM.SetAll(j);  
    delay(20);
  }

  // Fade out all outputs
  for(int j=maxBrightness;j>=0;j--){
    ShiftPWM.SetAll(j);  
    delay(20);
  }

  if (false) {
  // output sine pattern
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t val = (uint8_t)(((float)sin(millis() / 150.0 + i / 8.0 * 2.0 * PI) + 1) * 128);
//    sr.set(i, val);
  }
  }
}


// ------------------------------------------------------------------------------------------------------------------------------------------------


/****************************************************************************
 *  DCC Signal decoder for UNI16ARD board.
 *  Arduino New Pro Mini
 *  use NmraDcc library from http://mrrwa.org/
 *  homepage https://sites.google.com/site/sidloweb/
 *  e-shop http://dccdoma.eshop-zdarma.cz/
 *
 *  author Petr Šídlo
 *  date 2018-09-04
 *  revision 1.2
 *  license GPLv2
 */


const uint8_t SW_VERSION = 12;

const uint16_t CV_AUXILIARY_ACTIVATION = 2;
const uint8_t VALUE_AUXILIARY_ACTIVATION = 6;  // change this value to restore CV defaults after upload sketch

const uint16_t CV_DECODER_KEY = 15;
const uint8_t VALUE_DECODER_KEY = 0;        // unlocked decoder
const uint16_t CV_DECODER_LOCK = 16;
const uint8_t VALUE_DECODER_LOCK = 0;       // unlocked decoder

const uint16_t CV_ROCO_ADDRESS = 34;
const uint8_t VALUE_ROCO_ADDRESS = 1;      // 1 - ROCO address, 0 - LENZ address
const uint16_t CV_FADE_RATE = 39;
const uint8_t VALUE_FADE_RATE = 2;       // 0 - 7
const uint16_t CV_NUM_SIGNAL_NUMBER = 40;
const uint8_t VALUE_NUM_SIGNAL_NUMBER = 8;       // 1 - 8 
const uint16_t CV_ASPECT_LAG = 41;
const uint8_t VALUE_ASPECT_LAG = 1;       // 0 - 255   LAG × 0,128 s

const uint16_t CV_PROD_ID_1 = 47;
const uint8_t VALUE_PROD_ID_1 = 1;              // productID #1  
const uint16_t CV_PROD_ID_2 = 48;
const uint8_t VALUE_PROD_ID_2 = 1;              // productID #2
const uint16_t CV_PROD_ID_3 = 49;
const uint8_t VALUE_PROD_ID_3 = 1;              // productID #3
const uint16_t CV_PROD_ID_4 = 50;
const uint8_t VALUE_PROD_ID_4 = 4;              // productID #4

const int NUM_OUTPUTS = 32;
const int NUM_SIGNAL_MAST = 8;

const int SEGMENT_SIZE = 13;

const int IDX_LIGHT_0_YELLOW_UPPER = 0;
const int IDX_LIGHT_1_GREEN        = 1;
const int IDX_LIGHT_2_RED          = 2;
const int IDX_LIGHT_3_LUNAR        = 3;
const int IDX_LIGHT_4_YELLOW_LOWER = 4;
const int IDX_LIGHT_5_BLUE         = 5;
const int IDX_LIGHT_6_GREEN_STRIP  = 6;
const int IDX_LIGHT_7_YELLOW_STRIP = 7;
const int IDX_LIGHT_8_LUNAR_LOWER  = 8;
const int IDX_LIGHT_9_BACKWARD     = 9;
const int IDX_SIGNAL_SET           = 10;
const int IDX_DEFAULT_ASPECT       = 11;
const int IDX_NUMBER_OF_ADDRESS    = 12;

const byte ONA           = NUM_OUTPUTS; // OUTPUT_NOT_ASSIGNED


const uint16_t START_CV_OUTPUT = 128;
const uint16_t END_CV_OUTPUT = START_CV_OUTPUT + (SEGMENT_SIZE * NUM_SIGNAL_MAST - 1);

const uint16_t START_CV_ASPECT_TABLE = 768;

const uint8_t INIT_DECODER_ADDRESS = 100;   // ACCESSORY DECODER ADDRESS default
uint16_t thisDecoderAddress = 100;          // ACCESSORY DECODER ADDRESS
uint16_t maxDecoderAddress = 0;         

int signalMastNumberIdx[40] ;
int posNumber[40] ;

//   connect                 A   B   C   D   E   F   G   H   I   J   K   L   M   N   O   P
//   offset                  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
//const byte OUTPUT_PIN[] = { 19, 18,  9,  8,  7,  6,  5,  4, 10, 11, 12,  3, 14, 15, 16, 17 };
//   pins                   A5  A4  D9  D8  D7  D6  D5  D4 D10 D11 D12  D3  A0  A1  A2  A3

const byte SIGNAL_SET_CSD_BASIC         = 0 ;   // ČSD basic signal set 
const byte SIGNAL_SET_CSD_INTERMEDIATE  = 1 ;   // ČSD intermediate signal set 
const byte SIGNAL_SET_CSD_EMBEDDED      = 2 ;   // ČSD embedded signal set 
const byte SIGNAL_SET_SZDC_BASIC        = 3 ;   // SŽDC basic signal set 
const byte SIGNAL_SET_CSD_MECHANICAL    = 4 ;   // ČSD mechanical signal set 

const byte LIGHT_OFF          = 0;
const byte LIGHT_ON           = 1;
const byte LIGHT_BLINKING_54  = 2;
const byte LIGHT_BLINKING_108 = 3;
const byte LIGHT_BLINKING_45  = 4;
const byte LIGHT_BLINKING_22  = 5;

const byte BUBL_OFF = 0;
const byte BUBL_ON = 1;
const byte BUBL_BLINKING_54_OFF = 2;
const byte BUBL_BLINKING_108_OFF = 3;
const byte BUBL_BLINKING_45_OFF = 4;
const byte BUBL_BLINKING_22_OFF = 5;
const byte BUBL_BLINKING_54_ON = 6;
const byte BUBL_BLINKING_108_ON = 7;
const byte BUBL_BLINKING_45_ON = 8;
const byte BUBL_BLINKING_22_ON = 9;

const unsigned long BLINKING_TIME_54  =  556;
const unsigned long BLINKING_TIME_108 =  278;
const unsigned long BLINKING_TIME_45  =  667;
const unsigned long BLINKING_TIME_22  = 1333;

const byte FADE_TIME_LIGHT[11] = { 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66 } ;
const int FADE_COUNTER_LIGHT_1[11] = { /* -1 */ 0,  1, 1, 1, 1, 1, 1, 2, 3, 6, 9 };
const int FADE_COUNTER_LIGHT_2[11] = { /* -1 */ 1, 10, 7, 4, 3, 2, 2, 3, 4, 7, 10 };


unsigned long fadeTimeLight[11] = { 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66 } ;

byte signalMastLightYellowUpperOutput[NUM_SIGNAL_MAST] = { 0, 5, 10, 15, 20, 25, ONA, ONA };   // yellow upper
byte signalMastLightGreenOutput[NUM_SIGNAL_MAST]       = { 1, 6, 11, 16, 21, 26, ONA, ONA };   // green
byte signalMastLightRedOutput[NUM_SIGNAL_MAST]         = { 2, 7, 12, 17, 22, 27, ONA, ONA };   // red
byte signalMastLightLunarOutput[NUM_SIGNAL_MAST]       = { 3, 8, 13, 18, 23, 28, ONA, ONA };   // lunar
byte signalMastLightYellowLowerOutput[NUM_SIGNAL_MAST] = { 4, 9, 14, 19, 24, 29, ONA, ONA };   // yellow lower
byte signalMastLightBlueOutput[NUM_SIGNAL_MAST]        = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };   // blue
byte signalMastLightGreenStripOutput[NUM_SIGNAL_MAST]  = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };   // green strip
byte signalMastLightYellowStripOutput[NUM_SIGNAL_MAST] = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };   // yellow strip
byte signalMastLightLunarLowerOutput[NUM_SIGNAL_MAST]  = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };   // lunar lower
byte signalMastLightBackwardOutput[NUM_SIGNAL_MAST]    = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };   // backward

byte signalMastSignalSet[NUM_SIGNAL_MAST]    = { SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC };   // signal set

byte signalMastNumberAddress[NUM_SIGNAL_MAST]    = { 1, 1, 1, 1, 1, 1, 1, 1 };   // number of address

byte signalMastLightYellowUpperState[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };      // yellow upper
byte signalMastLightGreenState[NUM_SIGNAL_MAST]       = { 255, 255, 255, 255, 255, 255, 255, 255 };      // green
byte signalMastLightRedState[NUM_SIGNAL_MAST]         = { 255, 255, 255, 255, 255, 255, 255, 255 };      // red
byte signalMastLightLunarState[NUM_SIGNAL_MAST]       = { 255, 255, 255, 255, 255, 255, 255, 255 };      // lunar
byte signalMastLightYellowLowerState[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };      // yellow lower
byte signalMastLightBlueState[NUM_SIGNAL_MAST]        = { 255, 255, 255, 255, 255, 255, 255, 255 };      // blue
byte signalMastLightGreenStripState[NUM_SIGNAL_MAST]  = { 255, 255, 255, 255, 255, 255, 255, 255 };      // green strip
byte signalMastLightYellowStripState[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };      // yellow strip
byte signalMastLightLunarLowerState[NUM_SIGNAL_MAST]  = { 255, 255, 255, 255, 255, 255, 255, 255 };      // lunar lower
byte signalMastLightBackwardState[NUM_SIGNAL_MAST]    = { 255, 255, 255, 255, 255, 255, 255, 255 };      // backward

byte signalMastCurrentAspect[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };
byte signalMastDefaultAspectIdx[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };

byte lightState[NUM_OUTPUTS] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
byte bublState[NUM_OUTPUTS] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
int lightCounter[NUM_OUTPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long lightStartTimeBubl[NUM_OUTPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long lightElapsedTimeBubl[NUM_OUTPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

byte signalMastLastCode[NUM_SIGNAL_MAST]    = { 255, 255, 255, 255, 255, 255, 255, 255 };   // last code
unsigned long signalMastLastTime[NUM_SIGNAL_MAST]    = { 0, 0, 0, 0, 0, 0, 0, 0 };   // last time
boolean signalMastCodeChanged[NUM_SIGNAL_MAST]    = { true, true, true, true, true, true, true, true };   // code changed

int counterNrOutput = 0 ;
int counterNrSignalMast = 0 ;
unsigned long currentTime ;
unsigned long aspectLag ;


NmraDcc Dcc;
DCC_MSG Packet;

uint8_t lsb;
uint8_t msb;
uint8_t decoderKey;
uint8_t decoderLock;
uint8_t rocoAddress;
uint8_t numSignalNumber;
uint8_t fadeRate;

/**********************************************************************************
 * Setup Arduino.
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  // initialize the digital pins as an outputs
  pinMode(ACK_BUSY_PIN, OUTPUT);
  digitalWrite(ACK_BUSY_PIN, LOW);

  setupShiftPWM();
  Serial.println("setupShiftPWM Done.");
//  for (int i = 0; i < NUM_OUTPUTS; i++) {
//    pinMode(OUTPUT_PIN[i], OUTPUT);
//    digitalWrite(OUTPUT_PIN[i], LOW);
//  }
//
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(0, 2, 1);

  Serial.println("Dcc setup done.");

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.initAccessoryDecoder( MAN_ID_DIY, SW_VERSION, FLAGS_OUTPUT_ADDRESS_MODE, 0);

  Serial.println("Accessory setup done.");

  uint8_t cvVersion = Dcc.getCV(CV_AUXILIARY_ACTIVATION);
  if (cvVersion != VALUE_AUXILIARY_ACTIVATION) {
    setFactoryDefault();
  }

  initLocalVariables();
  Serial.println("Setup Done.");
}

/**************************************************************************
 * Main loop.
 */
void loop() {
  currentTime = millis();

  Dcc.process();

  processAspectCode(counterNrSignalMast) ;
  counterNrSignalMast++ ;
  if (counterNrSignalMast >= numSignalNumber) {
    counterNrSignalMast = 0 ;
  }

  processOutputLight(counterNrOutput);
  counterNrOutput++ ;
  if (counterNrOutput >= NUM_OUTPUTS) {
    counterNrOutput = 0 ;
  }
}

/**************************************************************************
 * 
 */
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {
  
  if (rocoAddress) {
    Addr = Addr + 4;
  }


  if ((Addr < thisDecoderAddress)
      || (Addr >= maxDecoderAddress)) {
     return ;
  }
  
  uint16_t idx = Addr - thisDecoderAddress ;

  signalMastChangePos(signalMastNumberIdx[idx], posNumber[idx], Direction) ;
  
}

/**************************************************************************
 *
 */
void setFactoryDefault() {

  struct CVPair {
    uint16_t CV;
    uint8_t Value;
  };

  CVPair FactoryDefaultCVs[] = {
      { CV_ACCESSORY_DECODER_ADDRESS_LSB, INIT_DECODER_ADDRESS },
      { CV_ACCESSORY_DECODER_ADDRESS_MSB, 0 },
      { CV_AUXILIARY_ACTIVATION, VALUE_AUXILIARY_ACTIVATION },
      { CV_DECODER_KEY, VALUE_DECODER_KEY },
      { CV_DECODER_LOCK, VALUE_DECODER_LOCK },

      { CV_ROCO_ADDRESS, VALUE_ROCO_ADDRESS },
      { CV_FADE_RATE, VALUE_FADE_RATE },
      { CV_NUM_SIGNAL_NUMBER, VALUE_NUM_SIGNAL_NUMBER },
      { CV_ASPECT_LAG, VALUE_ASPECT_LAG },
      
      { CV_PROD_ID_1, VALUE_PROD_ID_1 },
      { CV_PROD_ID_2, VALUE_PROD_ID_2 },
      { CV_PROD_ID_3, VALUE_PROD_ID_3 },
      { CV_PROD_ID_4, VALUE_PROD_ID_4 },
    };

  uint8_t FactoryDefaultCVSize = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

  if (Dcc.isSetCVReady()) {
    for (uint16_t i = 0; i < FactoryDefaultCVSize; i++) {
      Dcc.setCV(FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
    }
  }

  uint8_t signalMasts[] = {      
      0,   1,   2,  3, 4, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 0
      5,   6,   7,  8, 9, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 1
      10,  11,  12, 13, 14, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 2
      15,   16,   17, 18, 19, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 3
      20,   21,   22, 23, 24, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 4
      25,  26,  27, 28, 29, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 5
      ONA,  ONA,  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 6
      ONA,  ONA,  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0,  0, 5   // signal mast 7
  } ;
  
  if (Dcc.isSetCVReady()) {

    uint16_t OutputCV ;
    
    for (uint16_t i = 0; i < (NUM_SIGNAL_MAST * SEGMENT_SIZE); i++) {
      OutputCV = START_CV_OUTPUT + i ;
      Dcc.setCV(OutputCV, signalMasts[i]);
    }
  }

  if (Dcc.isSetCVReady()) {

    uint16_t OutputCV = START_CV_ASPECT_TABLE ;
    
    for (uint16_t i = 0; i < NUM_SIGNAL_MAST; i++) {
      for (uint16_t j = 0; j < 32; j++) {
        Dcc.setCV(OutputCV, j);
        OutputCV++ ;
      }
    }
  }

  initLocalVariables() ;   
}

/**********************************************************************************
 * Init local variables.
 */
void initLocalVariables() {

  lsb = Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB);
  msb = Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB);
  thisDecoderAddress = (msb << 8) | lsb;

  decoderKey = Dcc.getCV(CV_DECODER_KEY);
  decoderLock = Dcc.getCV(CV_DECODER_LOCK);

  rocoAddress = Dcc.getCV(CV_ROCO_ADDRESS);

  fadeRate = Dcc.getCV(CV_FADE_RATE);

  fadeRate = fadeRate & 0x07;
  for (int i = 0; i < 11; i++) {
    fadeTimeLight[i] = FADE_TIME_LIGHT[i] * fadeRate;
  }

  numSignalNumber = Dcc.getCV(CV_NUM_SIGNAL_NUMBER);
  aspectLag = Dcc.getCV(CV_ASPECT_LAG);
  aspectLag = aspectLag << 7;

  initLocalVariablesSignalMast();

}

/**********************************************************************************
 * Init local variables for signal mast.
 */
void initLocalVariablesSignalMast() {

  int counter = START_CV_OUTPUT;

  for (int i = 0; i < NUM_SIGNAL_MAST; i++) {
    signalMastLightYellowUpperOutput[i] = Dcc.getCV(counter);   // yellow upper
    counter++;
    signalMastLightGreenOutput[i]       = Dcc.getCV(counter);   // green
    counter++;
    signalMastLightRedOutput[i]         = Dcc.getCV(counter);   // red
    Serial.print("Red ["); Serial.print(i); Serial.print("] = "); Serial.println(signalMastLightRedOutput[i]);
    counter++;
    signalMastLightLunarOutput[i]       = Dcc.getCV(counter);   // lunar
    counter++;
    signalMastLightYellowLowerOutput[i] = Dcc.getCV(counter);   // yellow lower
    counter++;
    signalMastLightBlueOutput[i]        = Dcc.getCV(counter);   // blue
    counter++;
    signalMastLightGreenStripOutput[i]  = Dcc.getCV(counter);   // green strip
    counter++;
    signalMastLightYellowStripOutput[i] = Dcc.getCV(counter);   // yellow strip
    counter++;
    signalMastLightLunarLowerOutput[i]  = Dcc.getCV(counter);   // lunar lower
    counter++;
    signalMastLightBackwardOutput[i]    = Dcc.getCV(counter);   // backward
    counter++;

    signalMastSignalSet[i] = Dcc.getCV(counter); 
    counter++;

    byte defaultAspectIdx = Dcc.getCV(counter);
    counter++;

    signalMastDefaultAspectIdx[i] = defaultAspectIdx ;
    
    signalMastNumberAddress[i] = Dcc.getCV(counter); 
    counter++;
    
  }


  maxDecoderAddress = thisDecoderAddress ;
  for (int i = 0; i < numSignalNumber; i++) { 
    maxDecoderAddress = maxDecoderAddress + signalMastNumberAddress[i] ;
  }


  int idx = 0 ;
  for (int i = 0; i < numSignalNumber; i++) {               
    for (int j = 0; j < signalMastNumberAddress[i]; j++) {  
      signalMastNumberIdx[idx] = i ;
      posNumber[idx] = j ;
      
      signalMastChangePos(signalMastNumberIdx[idx], posNumber[idx], 1) ;
      signalMastChangePos(signalMastNumberIdx[idx], posNumber[idx], 0) ;
      
      idx++ ;
    }  
  }

  for (int i = 0; i < numSignalNumber; i++) {              
    for (int j = 0; j < 8; j++) { 

      int dir = bitRead(signalMastDefaultAspectIdx[i], j) ;
 
      signalMastChangePos(i, j, dir) ;

    }
  }
  
}


/**********************************************************************************
 *
 */
void processOutputLight(byte nrOutput) {

  lightElapsedTimeBubl[nrOutput] = currentTime - lightStartTimeBubl[nrOutput];

  if (lightElapsedTimeBubl[nrOutput] > 0xFFFFFFF0) {
    lightElapsedTimeBubl[nrOutput] = 0 ;  
  }

  switch (bublState[nrOutput]) {
  case BUBL_ON:
    processBublOn(nrOutput);
    break;
  case BUBL_OFF:
    processBublOff(nrOutput);
    break;
  case BUBL_BLINKING_54_ON:
    processBublBlinkingOn54(nrOutput);
    break;
  case BUBL_BLINKING_108_ON:
    processBublBlinkingOn108(nrOutput);
    break;
  case BUBL_BLINKING_45_ON:
    processBublBlinkingOn45(nrOutput);
    break;
  case BUBL_BLINKING_22_ON:
    processBublBlinkingOn22(nrOutput);
    break;
  case BUBL_BLINKING_54_OFF:
    processBublBlinkingOff54(nrOutput);
    break;
  case BUBL_BLINKING_108_OFF:
    processBublBlinkingOff108(nrOutput);
    break;
  case BUBL_BLINKING_45_OFF:
    processBublBlinkingOff45(nrOutput);
    break;
  case BUBL_BLINKING_22_OFF:
    processBublBlinkingOff22(nrOutput);
    break;
  }

}

/**********************************************************************************
 * 
 */
void signalMastChangePos(int nrSignalMast, uint16_t pos, uint8_t Direction) {

  byte newAspectIdx = signalMastLastCode[nrSignalMast] ;

  bitWrite(newAspectIdx, pos, Direction) ;

  switch (signalMastNumberAddress[nrSignalMast]) {
    case 1:
      newAspectIdx = newAspectIdx & 0x01 ;
      break;
    case 2:
      newAspectIdx = newAspectIdx & 0x03 ;
      break;
    case 3:
      newAspectIdx = newAspectIdx & 0x07 ;
      break;
    case 4:
      newAspectIdx = newAspectIdx & 0x0F ;
      break;
    default:
      newAspectIdx = newAspectIdx & 0x1F ;
      break ;
  }

  if (signalMastLastCode[nrSignalMast] == newAspectIdx) {
    return;
  }

  signalMastLastCode[nrSignalMast] = newAspectIdx ;
  signalMastLastTime[nrSignalMast] = currentTime ;
  signalMastCodeChanged[nrSignalMast] = true ;
    
}  

/**********************************************************************************
 *
 */
void processAspectCode(int nrSignalMast) {

  if (! signalMastCodeChanged[nrSignalMast]) {
    return ;
  }

  unsigned long aspectCodeElapsedTime = currentTime - signalMastLastTime[nrSignalMast];

  if (aspectCodeElapsedTime < aspectLag) {
    return ;  
  }
  
  byte newCode = signalMastLastCode[nrSignalMast] ;
  byte newAspect = aspectJmri(nrSignalMast, newCode) ;
  signalMastChangeAspect(nrSignalMast, newAspect) ;
  signalMastCodeChanged[nrSignalMast] = false ;
  
}

/**********************************************************************************
 *
 */
void signalMastChangeAspect(int nrSignalMast, byte newAspect) {

  if (signalMastCurrentAspect[nrSignalMast] == newAspect) {
    return;
  }
  
  switch (signalMastSignalSet[nrSignalMast]) {
    case SIGNAL_SET_CSD_BASIC :
      signalMastChangeAspectCsdBasic(nrSignalMast, newAspect) ;
      break ;
    case SIGNAL_SET_CSD_INTERMEDIATE :
      signalMastChangeAspectCsdIntermediate(nrSignalMast, newAspect) ;
      break ;
    case SIGNAL_SET_CSD_EMBEDDED :
      signalMastChangeAspectCsdEmbedded(nrSignalMast, newAspect) ;
      break ;
    case SIGNAL_SET_SZDC_BASIC :
       signalMastChangeAspectSzdcBasic(nrSignalMast, newAspect) ;
      break ;
    case SIGNAL_SET_CSD_MECHANICAL :
       signalMastChangeAspectCsdMechanical(nrSignalMast, newAspect) ;
      break ;
  }
}  
  
/**********************************************************************************
 *
 */
void signalMastChangeAspectCsdBasic(int nrSignalMast, byte newAspect) {
  Serial.print("Change mast "); Serial.print(nrSignalMast); Serial.print(" to aspect "); Serial.println(newAspect);
  signalMastCurrentAspect[nrSignalMast] = newAspect;

  switch (newAspect) {
    
  case 0: // Stůj
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 1: // Volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 2: // Výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 3: // Očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 4: // Očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 5: // Očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 6: // 40 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 7: // 40 a výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 8: // 40 a očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper 
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 9: // 40 a očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 10: // 40 a očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 11: // 60 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 12: // 60 a výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 13: // 60 a očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 14: // 60 a očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 15: // 60 a očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 16: // 80 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 17: // 80 a výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 18: // 80 a očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 19: // 80 a očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 20: // 80 a očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 21: // Opakovaná volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 22: // Opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 23: // Opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 24: // Opakovaná očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 25: // Opakovaná očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 27: // Posun zakázán
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_ON);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 28: // Posun dovolen
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 29: // Posun dovolen - nezabezpečený
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 30: // Opatrně na přivolávací návěst bez červené
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_54);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 31: // Opatrně na přivolávací návěst
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_54);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  };
}

/**********************************************************************************
 *
 */
void signalMastChangeAspectCsdIntermediate(int nrSignalMast, byte newAspect) {

  signalMastCurrentAspect[nrSignalMast] = newAspect;

  switch (newAspect) {
    
  case 0: // Stůj
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 1: // Volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 2: // Výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 3: // Očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 4: // 40 a opakovaná volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 5: // 60 a opakovaná volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 6: // 40 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 7: // 40 a výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 8: // 40 a očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 9: // 80 a opakovaná volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 10: // 40 a opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 11: // 60 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 12: // 40 a opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 13: // 40 a opakovaná očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 14: // 40 a opakovaná očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 15: // 60 a opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 16: // 80 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 17: // 60 a opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 18: // 60 a opakovaná očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 19: // 60 a opakovaná očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 20: // 80 a opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 21: // Opakovaná volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 22: // Opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 23: // Opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 24: // Opakovaná očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 25: // Opakovaná očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 26: // 80 a opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 28: // Posun dovolen
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 29: // 80 a opakovaná očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 30: // 80 a opakovaná očekávej 80
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_BLINKING_54);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_ON);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 31: // Opatrně na přivolávací návěst
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_54);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  };
}

/**********************************************************************************
 *
 */
void signalMastChangeAspectCsdEmbedded(int nrSignalMast, byte newAspect) {

  signalMastCurrentAspect[nrSignalMast] = newAspect;

  switch (newAspect) {
    
  case 0: // Stůj
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 1: // Volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 2: // Výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 3: // Očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 5: // Odjezdové návěstidlo dovoluje jízdu
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_22);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 6: // Stůj s modrou
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_ON);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 7: // 40 a výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 15: // Sunout zakázáno opakovaná
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_ON);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 16: // Sunout zakázáno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 17: // Sunout pomalu
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_ON);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 18: // Sunout rychleji
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 19: // Zpět
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_ON);      // backward

    break;

  case 20: // Zpět opakovaná
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_ON);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_ON);      // backward

    break;

  case 21: // Opakovaná volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 22: // Opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 23: // Opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 25: // Posun zakázán opakovaná
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 26: // Na spádovišti se neposunuje
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 27: // Posun zakázán
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_ON);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 28: // Posun dovolen
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 29: // Posun dovolen - nezabezpečený
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 31: // Opatrně na přivolávací návěst
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  };
}

/**********************************************************************************
 *
 */
void signalMastChangeAspectSzdcBasic(int nrSignalMast, byte newAspect) {

  signalMastCurrentAspect[nrSignalMast] = newAspect;

  switch (newAspect) {
    
  case 0: // Stůj
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 1: // Volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 2: // Výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 3: // Očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 4: // Očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 6: // 40 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 7: // 40 a výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 8: // 40 a očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 9: // 40 a očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 11: // 60 a volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 12: // 60 a výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 13: // 60 a očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 14: // 60 a očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_ON);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 16: // 40 a opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 17: // 40 a opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 18: // 40 a opakovaná očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 19: // Jízda podle rozhledových poměrů
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_54);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 20: // 40 a jízda podle rozhledových poměrů
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_54);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_ON);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 21: // Opakovaná volno
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_ON);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 22: // Opakovaná výstraha
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_ON);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 23: // Opakovaná očekávej 40
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_54);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 24: // Opakovaná očekávej 60
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_BLINKING_108);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 26: // Jízda vlaku dovolena
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_BLINKING_54);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 27: // Posun zakázán
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_ON);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 28: // Posun dovolen
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 29: // Posun dovolen - nezabezpečený
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 30: // Opatrně na přivolávací návěst bez červené
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_54);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 31: // Opatrně na přivolávací návěst
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_54);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  };
}

/**********************************************************************************
 *
 */
void signalMastChangeAspectCsdMechanical(int nrSignalMast, byte newAspect) {

  signalMastCurrentAspect[nrSignalMast] = newAspect;

  switch (newAspect) {
    
  case 0: // Stůj
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_ON);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_OFF);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 5: // Odjezdové návěstidlo dovoluje jízdu
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_BLINKING_22);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  case 28: // Posun dovolen
    changeSignalMastLightYellowUpperState(nrSignalMast, LIGHT_OFF);      // yellow upper
    changeSignalMastLightGreenState(nrSignalMast,       LIGHT_OFF);      // green
    changeSignalMastLightRedState(nrSignalMast,         LIGHT_OFF);      // red
    changeSignalMastLightLunarState(nrSignalMast,       LIGHT_ON);      // lunar
    changeSignalMastLightYellowLowerState(nrSignalMast, LIGHT_OFF);      // yellow lower
    changeSignalMastLightBlueState(nrSignalMast,        LIGHT_OFF);      // blue
    changeSignalMastLightGreenStripState(nrSignalMast,  LIGHT_OFF);      // green strip
    changeSignalMastLightYellowStripState(nrSignalMast, LIGHT_OFF);      // yellow strip
    changeSignalMastLightLunarLowerState(nrSignalMast,  LIGHT_OFF);      // lunar lower
    changeSignalMastLightBackwardState(nrSignalMast,    LIGHT_OFF);      // backward

    break;

  };
}

/**********************************************************************************
 *
 */
void changeSignalMastLightYellowUpperState(int nrSignalMast, byte newState) {

  if (signalMastLightYellowUpperState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightYellowUpperOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightYellowUpperState[nrSignalMast] = newState;

  changeLightState(signalMastLightYellowUpperOutput[nrSignalMast],
      signalMastLightYellowUpperState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightGreenState(int nrSignalMast, byte newState) {

  if (signalMastLightGreenState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightGreenOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightGreenState[nrSignalMast] = newState;

  changeLightState(signalMastLightGreenOutput[nrSignalMast],
      signalMastLightGreenState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightRedState(int nrSignalMast, byte newState) {

  if (signalMastLightRedState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightRedOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightRedState[nrSignalMast] = newState;

  changeLightState(signalMastLightRedOutput[nrSignalMast],
      signalMastLightRedState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightLunarState(int nrSignalMast, byte newState) {

  if (signalMastLightLunarState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightLunarOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightLunarState[nrSignalMast] = newState;

  changeLightState(signalMastLightLunarOutput[nrSignalMast],
      signalMastLightLunarState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightYellowLowerState(int nrSignalMast, byte newState) {

  if (signalMastLightYellowLowerState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightYellowLowerOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightYellowLowerState[nrSignalMast] = newState;

  changeLightState(signalMastLightYellowLowerOutput[nrSignalMast],
      signalMastLightYellowLowerState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightBlueState(int nrSignalMast, byte newState) {

  if (signalMastLightBlueState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightBlueOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightBlueState[nrSignalMast] = newState;

  changeLightState(signalMastLightBlueOutput[nrSignalMast],
      signalMastLightBlueState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightGreenStripState(int nrSignalMast, byte newState) {

  if (signalMastLightGreenStripState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightGreenStripOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightGreenStripState[nrSignalMast] = newState;

  changeLightState(signalMastLightGreenStripOutput[nrSignalMast],
      signalMastLightGreenStripState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightYellowStripState(int nrSignalMast, byte newState) {

  if (signalMastLightYellowStripState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightYellowStripOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightYellowStripState[nrSignalMast] = newState;

  changeLightState(signalMastLightYellowStripOutput[nrSignalMast],
      signalMastLightYellowStripState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightLunarLowerState(int nrSignalMast, byte newState) {

  if (signalMastLightLunarLowerState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightLunarLowerOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightLunarLowerState[nrSignalMast] = newState;

  changeLightState(signalMastLightLunarLowerOutput[nrSignalMast],
      signalMastLightLunarLowerState[nrSignalMast]);
}

/**********************************************************************************
 *
 */
void changeSignalMastLightBackwardState(int nrSignalMast, byte newState) {

  if (signalMastLightBackwardState[nrSignalMast] == newState) {
    return;
  }

  if (signalMastLightBackwardOutput[nrSignalMast] >= NUM_OUTPUTS) {
    return;
  }

  signalMastLightBackwardState[nrSignalMast] = newState;

  changeLightState(signalMastLightBackwardOutput[nrSignalMast],
      signalMastLightBackwardState[nrSignalMast]);
}



/**********************************************************************************
 *
 */
void changeLightState(byte lightOutput, byte newState) {

  if (lightState[lightOutput] == newState) {
    return;
  }

  switch (newState) {

  case LIGHT_OFF:
    if (bublState[lightOutput] == BUBL_ON 
        || bublState[lightOutput] == BUBL_BLINKING_54_ON 
        || bublState[lightOutput] == BUBL_BLINKING_108_ON 
        || bublState[lightOutput] == BUBL_BLINKING_45_ON 
        || bublState[lightOutput] == BUBL_BLINKING_22_ON) {
      bublState[lightOutput] = BUBL_OFF;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_OFF;
    } else { // (bublState[lightOutput] == BUBL_OFF || bublState[lightOutput] == BUBL_BLINKING_nnn_OFF)
      bublState[lightOutput] = BUBL_OFF;
      lightState[lightOutput] = LIGHT_OFF;
    }
    break;

  case LIGHT_ON:
    if (bublState[lightOutput] == BUBL_ON
        || bublState[lightOutput] == BUBL_BLINKING_54_ON 
        || bublState[lightOutput] == BUBL_BLINKING_108_ON 
        || bublState[lightOutput] == BUBL_BLINKING_45_ON 
        || bublState[lightOutput] == BUBL_BLINKING_22_ON) {
      bublState[lightOutput] = BUBL_ON;
      lightState[lightOutput] = LIGHT_ON;
    } else { // (bublState[lightOutput] == BUBL_OFF || bublState[lightOutput] == BUBL_BLINKING_nnn_OFF)
      bublState[lightOutput] = BUBL_ON;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_ON;
    }
    break;

  case LIGHT_BLINKING_54:
    if (bublState[lightOutput] == BUBL_ON
        || bublState[lightOutput] == BUBL_BLINKING_54_ON 
        || bublState[lightOutput] == BUBL_BLINKING_108_ON 
        || bublState[lightOutput] == BUBL_BLINKING_45_ON 
        || bublState[lightOutput] == BUBL_BLINKING_22_ON) {
      bublState[lightOutput] = BUBL_BLINKING_54_OFF;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_54;
    } else { // (bublState[lightOutput] == BUBL_OFF || bublState[lightOutput] == BUBL_BLINKING_nnn_OFF)
      bublState[lightOutput] = BUBL_BLINKING_54_ON;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_54;
    }
    break;

  case LIGHT_BLINKING_108:
    if (bublState[lightOutput] == BUBL_ON
        || bublState[lightOutput] == BUBL_BLINKING_54_ON 
        || bublState[lightOutput] == BUBL_BLINKING_108_ON 
        || bublState[lightOutput] == BUBL_BLINKING_45_ON 
        || bublState[lightOutput] == BUBL_BLINKING_22_ON) {
      bublState[lightOutput] = BUBL_BLINKING_108_OFF;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_108;
    } else { // (bublState[lightOutput] == BUBL_OFF || bublState[lightOutput] == BUBL_BLINKING_nnn_OFF)
      bublState[lightOutput] = BUBL_BLINKING_108_ON;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_108;
    }
    break;

  case LIGHT_BLINKING_45:
    if (bublState[lightOutput] == BUBL_ON
        || bublState[lightOutput] == BUBL_BLINKING_54_ON 
        || bublState[lightOutput] == BUBL_BLINKING_108_ON 
        || bublState[lightOutput] == BUBL_BLINKING_45_ON 
        || bublState[lightOutput] == BUBL_BLINKING_22_ON) {
      bublState[lightOutput] = BUBL_BLINKING_45_OFF;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_45;
    } else { // (bublState[lightOutput] == BUBL_OFF || bublState[lightOutput] == BUBL_BLINKING_nnn_OFF)
      bublState[lightOutput] = BUBL_BLINKING_45_ON;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_45;
    }
    break;

  case LIGHT_BLINKING_22:
    if (bublState[lightOutput] == BUBL_ON
        || bublState[lightOutput] == BUBL_BLINKING_54_ON 
        || bublState[lightOutput] == BUBL_BLINKING_108_ON 
        || bublState[lightOutput] == BUBL_BLINKING_45_ON 
        || bublState[lightOutput] == BUBL_BLINKING_22_ON) {
      bublState[lightOutput] = BUBL_BLINKING_22_OFF;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_22;
    } else { // (bublState[lightOutput] == BUBL_OFF || bublState[lightOutput] == BUBL_BLINKING_nnn_OFF)
      bublState[lightOutput] = BUBL_BLINKING_22_ON;
      lightStartTimeBubl[lightOutput] = millis();
      lightState[lightOutput] = LIGHT_BLINKING_22;
    }
    break;
  }

}

/**********************************************************************************
 *
 */
void processBublOn(byte nrOutput) {
  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {
    ShiftPWM.SetOne(nrOutput, maxBrightness);
    lightCounter[nrOutput] = 0;
    return;
  }

  processFadeOn(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublOff(byte nrOutput) {
  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {
    ShiftPWM.SetOne(nrOutput, minBrightness);
    lightCounter[nrOutput] = 0;
    return;
  }

  processFadeOff(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOn54(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, maxBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_54) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_54_OFF;    // = BUBL_BLINKING_OFF;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;
    }
    return;
  }

  processFadeOn(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOn108(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, maxBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_108) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_108_OFF;    // = BUBL_BLINKING_OFF;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;
    }
    return;
  }

  processFadeOn(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOn45(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, maxBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_45) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_45_OFF;    // = BUBL_BLINKING_OFF;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;
    }
    return;
  }

  processFadeOn(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOn22(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, maxBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_22) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_22_OFF;    // = BUBL_BLINKING_OFF;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;
    }
    return;
  }

  processFadeOn(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOff54(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, minBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_54) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_54_ON;    // = BUBL_BLINKING_ON;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;      
    }
    return;
  }

  processFadeOff(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOff108(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, minBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_108) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_108_ON;    // = BUBL_BLINKING_ON;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;
    }
    return;
  }

  processFadeOff(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOff45(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, minBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_45) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_45_ON;    // = BUBL_BLINKING_ON;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;
    }
    return;
  }

  processFadeOff(nrOutput);
}

/**********************************************************************************
 *
 */
void processBublBlinkingOff22(byte nrOutput) {

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {

    ShiftPWM.SetOne(nrOutput, minBrightness);

    if (lightElapsedTimeBubl[nrOutput] > BLINKING_TIME_22) { // if (elapsedTimeBubl > BLINKING_TIME) {
      bublState[nrOutput] = BUBL_BLINKING_22_ON;    // = BUBL_BLINKING_ON;
      lightStartTimeBubl[nrOutput] = millis();
      lightCounter[nrOutput] = 0;
    }
    return;
  }

  processFadeOff(nrOutput);
}

/**********************************************************************************
 *
 */
void processFadeOn(byte nrOutput) {

  int idx = 0;
  int limit = (sizeof(fadeTimeLight) / sizeof(fadeTimeLight[0]));

  for (idx = 0; idx < limit && lightElapsedTimeBubl[nrOutput] > fadeTimeLight[idx]; idx++) ;
  int span = (maxBrightness - minBrightness);
  int pwm = idx >= limit ? maxBrightness : ((span * FADE_COUNTER_LIGHT_1[idx]) / FADE_COUNTER_LIGHT_2[idx]) + minBrightness;

  ShiftPWM.SetOne(nrOutput, pwm);

//  Serial.print("Fade ON, idx="); Serial.print(idx); Serial.print(", counters: "); 
//  Serial.print(FADE_COUNTER_LIGHT_1[idx]); Serial.print("/"); Serial.print(FADE_COUNTER_LIGHT_2[idx]); 
//  Serial.print(", time="); Serial.print(lightElapsedTimeBubl[nrOutput]);
//  Serial.print(" pwm="); Serial.println(pwm);

/*
  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[11]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[11]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[9]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[10]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[10]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[8]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[9]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[9]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[7]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[8]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[8]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[6]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[7]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[7]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[5]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[6]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[6]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[4]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[5]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[5]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[3]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[4]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[4]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[2]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[3]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[3]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[1]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[2]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[2]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[0]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[1]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[1]) {
      lightCounter[nrOutput] = 0;
    }
  } else { if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[0]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[0]) {
      lightCounter[nrOutput] = 0;
    }
  }
*/
}

/**********************************************************************************
 *
 */
void processFadeOff(byte nrOutput) {
  int idx;
  int limit = (sizeof(fadeTimeLight) / sizeof(fadeTimeLight[0]));
  for (idx = 0; idx < limit && lightElapsedTimeBubl[nrOutput] > fadeTimeLight[idx]; idx++) ;
  
  int span = (maxBrightness - minBrightness);
  int pwm = idx >= limit ? minBrightness : maxBrightness - ((span * FADE_COUNTER_LIGHT_1[idx]) / FADE_COUNTER_LIGHT_2[idx]);

  ShiftPWM.SetOne(nrOutput, pwm);

//  Serial.print("Fade OFF, idx="); Serial.print(idx); Serial.print(", counters: "); 
//  Serial.print(FADE_COUNTER_LIGHT_1[idx]); Serial.print("/"); Serial.print(FADE_COUNTER_LIGHT_2[idx]); 
//  Serial.print(", time="); Serial.print(lightElapsedTimeBubl[nrOutput]);
//  Serial.print(" pwm="); Serial.println(pwm);
  /*
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[9]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[10]) {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[10]) {
      lightCounter[nrOutput] = 0;
    }

  if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[10]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[11]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[11]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[9]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[10]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[10]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[8]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[9]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[9]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[7]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[8]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[8]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[6]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[7]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[7]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[5]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[6]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[6]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[4]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[5]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[5]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[3]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[4]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[4]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[2]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[3]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[3]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[1]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[2]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[2]) {
      lightCounter[nrOutput] = 0;
    }
  } else if (lightElapsedTimeBubl[nrOutput] > fadeTimeLight[0]) {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[1]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[1]) {
      lightCounter[nrOutput] = 0;
    }
  } else {
    if (lightCounter[nrOutput] <= FADE_COUNTER_LIGHT_1[0]) {
      digitalWrite(OUTPUT_PIN[nrOutput], LOW);
    } else {
      digitalWrite(OUTPUT_PIN[nrOutput], HIGH);
    }
    lightCounter[nrOutput]++;
    if (lightCounter[nrOutput] >= FADE_COUNTER_LIGHT_2[0]) {
      lightCounter[nrOutput] = 0;
    }
  }
  */
}

/**************************************************************************
 *
 */
void notifyCVAck() {

  digitalWrite(ACK_BUSY_PIN, HIGH);
  delay(6);
  digitalWrite(ACK_BUSY_PIN, LOW);
}

/**************************************************************************
 * CV was changed.
 */
void notifyCVChange(uint16_t CV, uint8_t Value) {

  if (CV == CV_ACCESSORY_DECODER_ADDRESS_LSB) {
    lsb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return ;
  }

  if (CV == CV_ACCESSORY_DECODER_ADDRESS_MSB) {
    msb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return ;
  }

  if (CV == CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB) {
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, Value);
    lsb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return ;
  }

  if (CV == CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB) {
    Value = Value & 0x07;
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, Value);
    msb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return ;
  }

  if (CV == CV_ROCO_ADDRESS) {
    rocoAddress = Value;
    return ;
  }

  if (CV == CV_NUM_SIGNAL_NUMBER) {
    numSignalNumber = Value;
    return ;
  }

  if (CV == aspectLag) {
    aspectLag = Value;
    aspectLag = aspectLag << 7;
    return ;
  }

  if (CV == CV_DECODER_KEY) {
    decoderKey = Value;
    return ;
  }

  if (CV == CV_DECODER_LOCK) {
    decoderLock = Value;
    return ;
  }

  if (CV == CV_FADE_RATE) {
    fadeRate = Value;
    fadeRate = fadeRate & 0x07;
    for (int i = 0; i < 11; i++) {
      fadeTimeLight[i] = FADE_TIME_LIGHT[i] * fadeRate;
    }
    return ;
  }

  if (CV >= START_CV_OUTPUT && CV <= END_CV_OUTPUT) {
    Serial.print("CV changed: "); Serial.println(CV);
    initLocalVariablesSignalMast() ;
    return ;
  }

}

/******************************************************************************
 * Check CV valid.
 */
uint8_t notifyCVValid(uint16_t CV, uint8_t Writable) {

  uint8_t unlocked = isUnlocked(CV);

  if (unlocked && Writable && (CV == CV_MANUFACTURER_ID)) {
    setFactoryDefault();
    return false;
  }
  
  if (!unlocked) {
    return false;
  }

  if (CV > MAXCV) {
    return false;
  }

  if (unlocked && Writable && (CV == CV_VERSION_ID)) {
    return false;
  }

  return true;
}

/********************************************************************
 * Test for unlocked key.
 */
uint8_t isUnlocked(uint16_t CV) {

  if (CV == CV_DECODER_KEY) {
    return true;
  }

  if (decoderKey == 255) {
    return true;
  }

  if (decoderLock == decoderKey) {
    return true;
  }

  return false;
}

/********************************************************************
 * 
 */
byte aspectJmri(int nrSignalMast, byte aspectMx) {
    
    byte aspectJmri ;
    
    uint16_t cvAdr = START_CV_ASPECT_TABLE ;
    
    cvAdr = cvAdr + 32 * nrSignalMast + aspectMx ;
    
    aspectJmri = Dcc.getCV(cvAdr);

    return aspectJmri ;
}