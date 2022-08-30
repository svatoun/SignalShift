/****************************************************************************
 *  DCC Signal decoder for Arduino Nano / Arduino Pro Mini boards.

 *  use NmraDcc library from http://mrrwa.org/
 *
 *  Original code author: Petr Šídlo
 *  date 2018-09-04
 *  revision 1.2
 *  Original license GPLv2
 *  homepage https://sites.google.com/site/sidloweb/
 *
 *  Author modifications: svatopluk.dedic@gmail.com
 *  RELICENSED to Apache Public License 2.0 with the permission of the Original Author / Copyright holder.
 */

#include "Arduino.h"
#include <NmraDcc.h>

#include "Common.h"

// ------- Diagnostic setup -----------
const boolean debugFadeOnOff = false;
const boolean debugLightFlip = false;
const boolean debugLights = false;
const boolean debugAspects = false;

const uint8_t SW_VERSION = 12;

// --------- HARDWARE PIN WIRING / SETUP --------------------
const byte ACK_BUSY_PIN = 10;

// ShiftPWM HW setup setup
const byte ShiftPWM_dataPin = 11;
const byte ShiftPWM_clockPin = 13;
const byte ShiftPWM_latchPin = 8;

// min/max brightness for PWM. PWM is in range 0-255.
unsigned char maxBrightness = 255;
unsigned char minBrightness = 0;
unsigned char pwmFrequency = 100;
const bool ShiftPWM_balanceLoad = false;
const bool ShiftPWM_invertOutputs = true; 
#define SHIFTPWM_USE_TIMER2

// must be included after #define for timer
#include <ShiftPWM.h> 

/* ------------ Supported CV numbers ------------------ */
const uint16_t CV_AUXILIARY_ACTIVATION = 1;
const uint16_t CV_DECODER_KEY = 15;
const uint16_t CV_DECODER_LOCK = 16;
const uint16_t CV_ROCO_ADDRESS = 34;
const uint16_t CV_FADE_RATE = 39;
const uint16_t CV_NUM_SIGNAL_NUMBER = 40;
const uint16_t CV_ASPECT_LAG = 41;

const uint16_t CV_PROD_ID_1 = 47;
const uint16_t CV_PROD_ID_2 = 48;
const uint16_t CV_PROD_ID_3 = 49;
const uint16_t CV_PROD_ID_4 = 50;

/* ------------- Default values for supported CVs ---------- */

const uint8_t VALUE_AUXILIARY_ACTIVATION = 3;  // change this value to restore CV defaults after upload sketch
const uint8_t VALUE_DECODER_KEY = 0;        // unlocked decoder
const uint8_t VALUE_DECODER_LOCK = 0;       // unlocked decoder

const uint8_t VALUE_ROCO_ADDRESS = 1;      // 1 - ROCO address, 0 - LENZ address
const uint8_t VALUE_FADE_RATE = 2;       // 0 - 7
const uint8_t VALUE_NUM_SIGNAL_NUMBER = 8;       // 1 - 8 
const uint8_t VALUE_ASPECT_LAG = 1;       // 0 - 255   LAG × 0,128 s

const uint8_t VALUE_PROD_ID_1 = 1;              // productID #1  
const uint8_t VALUE_PROD_ID_2 = 1;              // productID #2
const uint8_t VALUE_PROD_ID_3 = 1;              // productID #3
const uint8_t VALUE_PROD_ID_4 = 4;              // productID #4

/**
 * Maximum possible outputs (lights) for a single mast. The value here affects the CV layout: for each Mast,
 * a consecutive CV list specifies physical outputs of individual Mast lights. Changing the value here will
 * damage existing decoder configurations as the CVs will be shifted.
 */
const int maxOutputsPerMast = 10;

/**
 * Maximum aspects supported per one signal table.
 */
const int maxAspects = 32;

/**
 * Maximum number of outputs. This value does not affect the CV layout, but is used as a dimension of status arrays.
 */
const int NUM_OUTPUTS = 80;

/**
 * Maximum number of signal masts supported by a single decoder.
 */
const int NUM_SIGNAL_MAST = 16;

const int NUM_8BIT_SHIFT_REGISTERS = (NUM_OUTPUTS + 7) / 8;

const int SEGMENT_SIZE = maxOutputsPerMast + 3;

const int maxAspectBits = 5;
static_assert (maxAspects <= (1 << maxAspectBits), "Too many aspects, do not fit in 5 bits");

const byte ONA           = NUM_OUTPUTS; // OUTPUT_NOT_ASSIGNED

const uint16_t START_CV_OUTPUT = 128;
const uint16_t END_CV_OUTPUT = START_CV_OUTPUT + (SEGMENT_SIZE * NUM_SIGNAL_MAST - 1);

const uint16_t START_CV_ASPECT_TABLE = 512;

const uint8_t INIT_DECODER_ADDRESS = 100;   // ACCESSORY DECODER ADDRESS default
uint16_t thisDecoderAddress = 100;          // ACCESSORY DECODER ADDRESS
uint16_t maxDecoderAddress = 0;         

int signalMastNumberIdx[40] ;
int posNumber[40] ;

//   connect                 A   B   C   D   E   F   G   H   I   J   K   L   M   N   O   P
//   offset                  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
//const byte OUTPUT_PIN[] = { 19, 18,  9,  8,  7,  6,  5,  4, 10, 11, 12,  3, 14, 15, 16, 17 };
//   pins                   A5  A4  D9  D8  D7  D6  D5  D4 D10 D11 D12  D3  A0  A1  A2  A3

// Signal sets defined in the decoder
enum SignalSet : byte {
  SIGNAL_SET_CSD_BASIC         = 0,  // ČSD basic signal set 
  SIGNAL_SET_CSD_INTERMEDIATE  = 1,   // ČSD intermediate signal set 
  SIGNAL_SET_CSD_EMBEDDED      = 2,   // ČSD embedded signal set 
  SIGNAL_SET_SZDC_BASIC        = 3,   // SŽDC basic signal set 
  SIGNAL_SET_CSD_MECHANICAL    = 4,   // ČSD mechanical signal set 

  _signal_set_last // must be last
};

// Signs on an individual light. Fixed (on, off) and blinking.
enum LightSign : byte {
  inactive = 0x00,      // since initialized globals are zeroed after initializer till end of object
  fixed = 0x01,
  blinking54,
  blinking108,
  blinking45,
  blinking22,

  _lightsign_last   // must be last
};

#define LON (fixed)
#define LOFF (fixed | 0x10)
#define L(n) (n)
// originally, blinking had a bit flag, but that's not necessary
#define B(n) (n)


/**
 * Describes the state of a light output. Defined as a structure for
 * easier access; the alternative would be a bit manipulation.
 */
struct LightFunction {
  /**
   * The sign signalled by the light. This field 
   */
  LightSign sign : 4;
  static_assert (_lightsign_last <= 16, "Too many signs, must fit in 4 bits (LightFunction::sign)");

 /**
   * True to change the light towards OFF, false to change towards ON.
   */
  boolean off : 1;

  /**
   * If true, the light reached the on/off terminal state
   */
  boolean end : 1;

  // Default initializer: output off(=true), final brightness (end=true), not alternating
  LightFunction() : sign(inactive), off(true), end(true) {}

  // Normal construction
  LightFunction(LightSign sign, boolean initOff) : sign(sign), off(initOff), end(false) {}

  LightFunction(byte data) { *((byte*)(void*)this) = data; }

  // Copy constructor for easy assignment; make fast assingment of the whole byte.
  LightFunction(const LightFunction& f)  { *((byte*)(void*)this) = *((byte*)(void*)&f); }
};

const unsigned long BLINKING_TIME_54  =  556;
const unsigned long BLINKING_TIME_108 =  278;
const unsigned long BLINKING_TIME_45  =  667;
const unsigned long BLINKING_TIME_22  = 1333;

// PENDING: The brightness of LED is perceived non-lineraly by human eye. Revise the scale
// here according to https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
const byte FADE_TIME_LIGHT[11] = { 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66 } ;
const int FADE_COUNTER_LIGHT_1[11] = { /* -1 */ 0,  1, 1, 1, 1, 1, 1, 2, 3, 6, 9 };
const int FADE_COUNTER_LIGHT_2[11] = { /* -1 */ 1, 10, 7, 4, 3, 2, 2, 3, 4, 7, 10 };

// Will be initialized at startup, and whenever fadeRate global changes. As fadeRate is max 7, the value nicely fits in 9 bits unsigned.
unsigned int fadeTimeLight[11] = { } ;

struct MastConfig {
  byte  outputs[maxOutputsPerMast];
  byte  signalSet;
  byte  defaultAspect;
  byte  numberOfAddresses;

/*
  MastConfig(const byte (&aOutputs)[maxOutputsPerMast], byte aSignalSet, byte aDefaultAspect, byte aNumberOfAddresses) : signalSet(aSignalSet), defaultAspect(aDefaultAspect), numberOfAddresses(aNumberOfAddresses) {
    memcpy(outputs, aOutputs, sizeof(outputs));
  }

  MastConfig(const MastConfig& other) : signalSet(other.signalSet), defaultAspect(other.defaultAspect), numberOfAddresses(other.numberOfAddresses) {
    memcpy(outputs, other.outputs, sizeof(outputs));
  }
*/
};
static_assert (sizeof(MastConfig) == SEGMENT_SIZE, "Mismatch in MastConfig / SEGMENT_SIZE");

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

typedef byte oneLightOutputs[NUM_SIGNAL_MAST];

oneLightOutputs * lightConfiguration[maxOutputsPerMast] = {
  &signalMastLightYellowUpperOutput,
  &signalMastLightGreenOutput,
  &signalMastLightRedOutput,
  &signalMastLightLunarOutput,
  &signalMastLightYellowLowerOutput,
  &signalMastLightBlueOutput,
  &signalMastLightGreenStripOutput,
  &signalMastLightYellowStripOutput,
  &signalMastLightLunarLowerOutput,
  &signalMastLightBackwardOutput,
};

SignalSet signalMastSignalSet[NUM_SIGNAL_MAST]    = { SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC };   // signal set

byte signalMastNumberAddress[NUM_SIGNAL_MAST]    = { 1, 1, 1, 1, 1, 1, 1, 1 };   // number of address

byte signalMastCurrentAspect[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };
byte signalMastDefaultAspectIdx[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };

LightFunction bublState2[NUM_OUTPUTS] = {  };

unsigned int lightStartTimeBubl[NUM_OUTPUTS] = {  };

byte signalMastLastCode[NUM_SIGNAL_MAST]    = { 255, 255, 255, 255, 255, 255, 255, 255 };   // last code
unsigned long signalMastLastTime[NUM_SIGNAL_MAST]    = { };   // last time
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
  Serial.print(F("UNI16ARD-NAV-Shift version ")); Serial.println(SW_VERSION); 
  Serial.println(F("Copyright (c) 2018, Petr Sidlo <sidlo64@seznam.cz>\nCopyright (c) 2022, Svatopluk Dedic <svatopluk.dedic@gmail.com>, APL 2.0 License"));
  Serial.println("Booting...");

  // initialize the digital pins as an outputs
  pinMode(ACK_BUSY_PIN, OUTPUT);
  digitalWrite(ACK_BUSY_PIN, LOW);

  setupShiftPWM();
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(0, 2, 1);

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.initAccessoryDecoder( MAN_ID_DIY, SW_VERSION, FLAGS_OUTPUT_ADDRESS_MODE, 0);

  uint8_t cvVersion = Dcc.getCV(CV_AUXILIARY_ACTIVATION);
  if (cvVersion != VALUE_AUXILIARY_ACTIVATION) {
    setFactoryDefault();
  }

  initLocalVariables();

  ModuleChain::invokeAll(initialize);

  Serial.print(F("Driving max ")); Serial.print(NUM_OUTPUTS); Serial.print(F(" outputs from max ")); Serial.print(NUM_SIGNAL_MAST); Serial.println(F(" masts"));

  initTerminal();
  setupTerminal();
}

void setupShiftPWM() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);

  pinMode(2, INPUT);

  ShiftPWM.SetAmountOfRegisters(8);
  ShiftPWM.SetPinGrouping(1); //This is the default, but I added here to demonstrate how to use the funtion
  ShiftPWM.Start(pwmFrequency,maxBrightness);
  ShiftPWM.SetAll(0);
}


/**************************************************************************
 * Main loop.
 */
void loop() {
  processTerminal();
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
struct CVPair {
  const uint16_t CV;
  const uint8_t Value;
};

const CVPair FactoryDefaultCVs[] = {
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

// must be represented as bytes, as array initialized by MastConfig initializers cannot be stored in PROGMEM.
const MastConfig initMasts[1] PROGMEM = {
  { { 0,   1,   2,  3, 4, ONA, ONA, ONA, ONA, ONA }, 0,  0, 5 }
};

const uint8_t factorySignalMastOutputs[] PROGMEM = {      
    0,   1,   2,  3, 4, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 0
    5,   6,   7,  8, 9, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 1
    10,  11,  12, 13, 14, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 2
    15,   16,   17, 18, 19, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 3
    20,   21,   22, 23, 24, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 4
    25,  26,  27, 28, 29, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 5
    ONA,  ONA,  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0,  0, 5,  // signal mast 6
    ONA,  ONA,  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0,  0, 5   // signal mast 7
} ;

const uint8_t noSignalMastOutputs[] PROGMEM = {    
    ONA,  ONA,  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0,  0, 5   // signal mast 7
};

static_assert (sizeof(factorySignalMastOutputs) == sizeof(MastConfig) * 8, "Inconsistency between MastConfig and init data");

void setFactoryDefault() {
  Serial.println(F("Resetting to factory defaults"));
  uint8_t FactoryDefaultCVSize = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

  for (uint16_t i = 0; i < FactoryDefaultCVSize; i++) {
    Dcc.setCV(FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  }

  int cvNumber = START_CV_OUTPUT;
  for (int i = 0; i < sizeof(factorySignalMastOutputs); i++, cvNumber++) {
    Dcc.setCV(cvNumber, pgm_read_byte_near(factorySignalMastOutputs + i));
  }

  for (int i = sizeof(factorySignalMastOutputs) / SEGMENT_SIZE; i < NUM_SIGNAL_MAST; i++) {
    for (int j = 0; j < sizeof(noSignalMastOutputs); j++, cvNumber++) {
      Dcc.setCV(cvNumber, pgm_read_byte_near(noSignalMastOutputs + j));
    }
  }

  uint16_t OutputCV = START_CV_ASPECT_TABLE ;
  
  for (uint16_t i = 0; i < NUM_SIGNAL_MAST; i++) {
    for (uint16_t j = 0; j < 32; j++) {
      Dcc.setCV(OutputCV, j);
      OutputCV++ ;
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
  initializeFadeTime();

  numSignalNumber = Dcc.getCV(CV_NUM_SIGNAL_NUMBER);
  aspectLag = Dcc.getCV(CV_ASPECT_LAG);
  aspectLag = aspectLag << 7;

  initLocalVariablesSignalMast();

}

void initializeFadeTime() {
  fadeRate = fadeRate & 0x07;
  for (int i = 0; i < 11; i++) {
    fadeTimeLight[i] = FADE_TIME_LIGHT[i] * fadeRate;
  }
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
    int addrs = signalMastNumberAddress[i];
    if (addrs > maxAspectBits) {
      Serial.print(F("Mast #")); Serial.print(i); Serial.print(F(" Invalid number of addresses: ")); Serial.println(addrs);
      addrs = 1;
    }
    for (int j = 0; j < addrs; j++) {  
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

int currentBulbTimeSpan = -1;

unsigned int timeElapsedForBulb(byte nrOutput) {
  unsigned int start = (lightStartTimeBubl[nrOutput] & 0xffff);
  if (start == 0) {
    return currentBulbTimeSpan = UINT16_MAX;
  }
  unsigned int cur = currentTime & 0xffff;
  int span;

  if (start > cur) {
    span = (unsigned int)((0x10000 - start) + cur);
  } else {
    span = cur - start;
  }
  if (span >= INT16_MAX) {
    return currentBulbTimeSpan = INT16_MAX;
  } else {
    return currentBulbTimeSpan = (int)span;
  }
}


/**********************************************************************************
 *
 */
void processOutputLight(byte nrOutput) {
  switch (bublState2[nrOutput].sign) {
    case fixed:
        processBulbBlinking(nrOutput, 0);
        break;
    case blinking54:
        processBulbBlinking(nrOutput, BLINKING_TIME_54);
        break;
    case blinking108:
        processBulbBlinking(nrOutput, BLINKING_TIME_108);
        break;
    case blinking45:
        processBulbBlinking(nrOutput, BLINKING_TIME_45);
        break;
    case blinking22:
        processBulbBlinking(nrOutput, BLINKING_TIME_22);
        break;
  }
}

void changeLightState2(byte lightOutput, struct LightFunction newState) {
  if (lightOutput >= NUM_OUTPUTS) {
    return;
  }
  LightFunction& bs = bublState2[lightOutput];
  if (debugLights) {
    Serial.print(F("Change light #")); Serial.print(lightOutput); Serial.print(F(" curState: ")); Serial.print(bs.sign); Serial.print(F(", off = ")); Serial.print(bs.off); Serial.print(F(", newstate: ")); Serial.print(newState.sign); Serial.print(F(", off = ")); Serial.println(newState.off); 
  }

  boolean wasOff = bs.off;
  if (bs.sign == newState.sign) {
    // exception: fixed(on) != fixed(off)
    if (newState.sign != fixed || newState.off == wasOff) {
      return;
    }
  }
  
  resetStartTime(lightOutput);
  bs = newState;
  if (newState.sign != fixed) {
    bs.off = !wasOff;
  }
  if (debugLights) {
    Serial.print(F("Change light #")); Serial.print(lightOutput);Serial.print(F(" newState: ")); Serial.print(bublState2[lightOutput].sign); Serial.print(F(", off = ")); Serial.println(bublState2[lightOutput].off); 
  }
}

void processBulbBlinking(byte nrOutput, int blinkDelay) {
  boolean off = bublState2[nrOutput].off;
  int elapsed = timeElapsedForBulb(nrOutput);
  if (elapsed > 0xff00) {
    // just to be sure, elapsed time is too large, ignore the light.
    return;
  }
  if (elapsed > fadeTimeLight[10]) {
    if (!bublState2[nrOutput].end) {
      if (debugLightFlip) {
        Serial.print("Light elapsed "); Serial.print(nrOutput); Serial.print(F( " off = ")); Serial.println(off);
      }
      bublState2[nrOutput].end = true;
      ShiftPWM.SetOne(nrOutput, off ? minBrightness : maxBrightness);
    }

    if (blinkDelay > 0) {
      if (elapsed > blinkDelay) {
        if (debugLightFlip) {
          Serial.print("Light blinked "); Serial.println(nrOutput); 
        }
        bublState2[nrOutput].end = false;
        bublState2[nrOutput].off = !off;
        resetStartTime(nrOutput);
      }
    } else {
      // disable state changes.
      if (debugLightFlip) {
        Serial.print("Light fixed "); Serial.print(nrOutput); Serial.print(F(" state ")); Serial.println(!off);
      }
      lightStartTimeBubl[nrOutput] = 0;
    }
    return;
  }
  if (off) {
    processFadeOff(nrOutput);
  } else {
    processFadeOn(nrOutput);
  }
}


/**********************************************************************************
 * 
 */
void signalMastChangePos(int nrSignalMast, uint16_t pos, uint8_t Direction) {
  if (nrSignalMast >= NUM_SIGNAL_MAST) {
    Serial.print(F("Error: mastChangePos mast out of range: ")); Serial.println(nrSignalMast);
    return;
  }
  if (pos >= maxOutputsPerMast) {
    Serial.print(F("Error: mastChangePos pos out of range")); Serial.println(pos);
    return;
  }
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

  if (newAspectIdx >= maxAspects) {
    Serial.print(F("Error: newAspectIdx out of range")); Serial.println(newAspectIdx);
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
typedef LightFunction AspectDefinition[maxOutputsPerMast];
typedef byte AspectDefinitionBytes[maxOutputsPerMast];
typedef AspectDefinitionBytes SignalSet32[32];

#define STRIP_OFF LOFF, LOFF, LOFF, LOFF, LOFF  
#define STRIP_40  STRIP_OFF
#define STRIP_60  LOFF, LOFF, LON,  LOFF, LOFF
#define STRIP_80  LOFF, LON,  LOFF, LOFF, LOFF  
#define B54 B(blinking54)
#define B108 B(blinking108)
#define B22 B(blinking22)
#define L___ LOFF

const SignalSet32 csdBasicAspects PROGMEM = {
  {   L___, L___, LON,  L___, L___,             STRIP_OFF                     },    // Aspect 0: Stuj
  {   L___, LON,  L___, L___, L___,             STRIP_OFF                     },    // Aspect 1: Volno
  {   LON,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 2: Výstraha,
  {   B54,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 3: Očekávej 40
  {   B108, L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 4: Očekávej 60
  {   L___, B54,  L___, L___, L___,             STRIP_OFF                     },    // Aspect 5: Očekávej 80
  {   L___, L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 6: 40 a volno
  {   LON,  L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 7: 40 a výstraha
  {   B54,  L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 8: 40 a očekávej 40
  {   B108, L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 9: 40 a očekávej 60
  {   L___, B54,  L___, L___, LON,              STRIP_OFF                     },    // Aspect 10: 40 a očekávej 80
  {   L___, LON,  L___, L___, LON,              STRIP_60                      },    // Aspect 11: 60 a volno
  {   LON,  L___, L___, L___, LON,              STRIP_60                      },    // Aspect 12: 60 a výstraha
  {   B54,  L___, L___, L___, LON,              STRIP_60                      },    // Aspect 13: 60 a očekávej 40
  {   B108, L___, L___, L___, LON,              STRIP_60                      },    // Aspect 14: 60 a očekávej 60
  {   L___, B54,  L___, L___, LON,              STRIP_60                      },    // Aspect 15: 60 a očekávej 80
  {   L___, LON,  L___, L___, LON,              STRIP_80                      },    // Aspect 16: 80 a volno
  {   LON,  L___, L___, L___, LON,              STRIP_80                      },    // Aspect 17: 80 a výstraha
  {   B54,  L___, L___, L___, LON,              STRIP_80                      },    // Aspect 18: 80 a očekávej 40
  {   B108, L___, L___, L___, LON,              STRIP_80                      },    // Aspect 19: 80 a očekávej 60
  {   L___, B54,  L___, L___, LON,              STRIP_80                      },    // Aspect 20: 80 a očekávej 80
  {   L___, LON,  L___, LON,  L___,             STRIP_OFF                     },    // Aspect 21: Opakovaná volno
  {   LON,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 22: Opakovaná výstraha
  {   B54,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 23: Opakovaná očekávej 40
  {   B108, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 24: Opakovaná očekávej 60
  {   L___, B54,  L___, LON,  L___,             STRIP_OFF                     },    // Aspect 25: Opakovaná očekávej 80
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 26: -----------------------
  {   STRIP_OFF,                                LON,  L___, L___, L___, L___  },    // Aspect 27: Posun zakázán
  {   L___, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 28: Posun dovolen
  {   L___, L___, LON,  LON,   L___,            STRIP_OFF                     },    // Aspect 29: Posun dovolen - nezabezpečený
  {   L___, L___, L___, B54,  L___,             STRIP_OFF                     },    // Aspect 30: Opatrně na přivolávací návěst bez červené
  {   L___, L___, LON,  B54,  L___,             STRIP_OFF                     },    // Aspect 31: Opatrně na přivolávací návěst
};

const SignalSet32 csdIntermediateAspects PROGMEM = {
  {   L___, L___, LON,  L___, L___,             STRIP_OFF                     },    // Aspect 0: Stuj
  {   L___, LON,  L___, L___, L___,             STRIP_OFF                     },    // Aspect 1: Volno
  {   LON,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 2: Výstraha,
  {   B54,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 3: Očekávej 40
  {   L___, LON,  L___, LON,  LON,              STRIP_OFF                     },    // Aspect 4: 40 a opakovaná volno             *
  {   L___, LON,  L___, LON,  LON,              STRIP_60                      },    // Aspect 5: 60 a opakovaná volno             *
  {   L___, L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 6: 40 a volno
  {   LON,  L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 7: 40 a výstraha
  {   B54,  L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 8: 40 a očekávej 40
  {   L___, LON,  L___, LON,  LON,              STRIP_80                      },    // Aspect 9: 80 a opakovaná volno             *
  {   LON,  L___, L___, LON,  LON,              STRIP_OFF                     },    // Aspect 10: 40 a opakovaná výstraha         *
  {   L___, LON,  L___, L___, LON,              STRIP_60                      },    // Aspect 11: 60 a volno
  {   B54,  L___, L___, LON,  LON,              STRIP_OFF                     },    // Aspect 12: 40 a opakovaná očekávej 40      *
  {   B108, L___, L___, LON,  LON,              STRIP_OFF                     },    // Aspect 13: 40 a opakovaná očekávej 60      *
  {   L___, B54,  L___, L___, LON,              STRIP_OFF                     },    // Aspect 14: 40 a opakovaná očekávej 80      *
  {   LON,  L___, L___, LON,  LON,              STRIP_60                      },    // Aspect 15: 60 a opakovaná výstraha         *
  {   L___, LON,  L___, L___, LON,              STRIP_80                      },    // Aspect 16: 80 a volno
  {   B54,  L___, L___, L___, LON,              STRIP_60                      },    // Aspect 17: 60 a opakovaná očekávej 40      *
  {   B108, L___, L___, L___, LON,              STRIP_60                      },    // Aspect 18: 60 a opakovaná očekávej 60      *
  {   L___, B54,  L___, L___, LON,              STRIP_60                      },    // Aspect 19: 60 a opakovaná očekávej 80      *
  {   LON,  L___, L___, LON,  LON,              STRIP_80                      },    // Aspect 20: 80 a opakovaná výstraha         *
  {   L___, LON,  L___, LON,  L___,             STRIP_OFF                     },    // Aspect 21: Opakovaná volno
  {   LON,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 22: Opakovaná výstraha
  {   B54,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 23: Opakovaná očekávej 40
  {   B108, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 24: Opakovaná očekávej 60
  {   L___, B54,  L___, LON,  L___,             STRIP_OFF                     },    // Aspect 25: Opakovaná očekávej 80
  {   B54,  L___, L___, LON,  LON,              STRIP_80                      },    // Aspect 26: 80 a opakovaná očekávej 40      *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 27: ----------------------------
  {   L___, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 28: Posun dovolen
  {   B108, L___, L___, LON,  LON,              STRIP_80                      },    // Aspect 29: 80 a opakovaná očekávej 60
  {   L___, L___, L___, B54,  L___,             STRIP_80                      },    // Aspect 30: 80 a opakovaná očekávej 80
  {   L___, L___, LON,  B54,  L___,             STRIP_OFF                     },    // Opatrně na přivolávací návěst
};

const SignalSet32 csdEmbeddedAspects PROGMEM = {
  {   L___, L___, LON,  L___, L___,             STRIP_OFF                     },    // Aspect 0: Stuj
  {   L___, LON,  L___, L___, L___,             STRIP_OFF                     },    // Aspect 1: Volno
  {   LON,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 2: Výstraha,
  {   B54,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 3: Očekávej 40
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 4: ----------------------------           *
  {   L___, L___, B22, L___,                    STRIP_OFF                     },    // Aspect 5: Odjezdové návěstidlo dovoluje jízdu    *
  {   L___, LON, L___, L___, L___,              LON,   L___, L___, L___, L___ },    // Aspect 6: Stůj s modrou                          *
  {   LON,  L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 7: 40 a výstraha
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 8:  ---------------------------           *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 9:  ---------------------------           *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 10: ---------------------------           *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 11: ---------------------------           *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 12: ---------------------------           *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 13: ---------------------------           *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 14: ---------------------------           *
  {   STRIP_OFF,                                LON,   L___, L___, L___, L___ },    // Aspect 15: Sunout zakázáno opakovaná             *
  {   L___, LON,  L___, L___, L___,             STRIP_OFF                     },    // Aspect 16: Sunout zakázáno                       *
  {   L___, L___, L___, LON, L___,              L___,  L___, L___, LON,  L___ },    // Aspect 17: Sunout pomalu                         *
  {   L___, L___, L___, LON, L___,              STRIP_OFF                     },    // Aspect 18: Sunout rychleji                       *
  {   L___, LON, L___, L___, L___,              L___,  L___, L___, L___, LON  },    // Aspect 19: Zpět                                  *
  {   STRIP_OFF,                                LON,   L___, L___, L___, LON  },    // Aspect 20: Zpět opakovaná                        *
  {   L___, LON,  L___, LON,  L___,             STRIP_OFF                     },    // Aspect 21: Opakovaná volno
  {   LON,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 22: Opakovaná výstraha
  {   B54,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 23: Opakovaná očekávej 40
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 24: -----------------------               *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 25: Posun zakázán opakovaná               *
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 26: Na spádovišti se neposunuje           *
  {   L___, L___, L___, L___, L___,             LON,  L___, L___, L___, L___  },    // Aspect 27: Posun zakázán
  {   L___, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 28: Posun dovolen
  {   L___, L___, LON, LON,   L___,             STRIP_OFF                     },    // Aspect 29: Posun dovolen - nezabezpečený
  {   L___, L___, L___, B54,  L___,             STRIP_OFF                     },    // Aspect 30: Opatrně na přivolávací návěst bez červené
  {   L___, L___, LON,  B54,  L___,             STRIP_OFF                     },    // Aspect 31: Opatrně na přivolávací návěst
};

const SignalSet32 szdcBasicAspects PROGMEM = {
  {   L___, L___, LON,  L___, L___,             STRIP_OFF                     },    // Aspect 0: Stuj
  {   L___, LON,  L___, L___, L___,             STRIP_OFF                     },    // Aspect 1: Volno
  {   LON,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 2: Výstraha,
  {   B54,  L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 3: Očekávej 40
  {   B108, L___, L___, L___, L___,             STRIP_OFF                     },    // Aspect 4: Očekávej 60
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 5: -----------------------                *
  {   L___, L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 6: 40 a volno
  {   LON,  L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 7: 40 a výstraha
  {   B54,  L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 8: 40 a očekávej 40
  {   B108, L___, L___, L___, LON,              STRIP_OFF                     },    // Aspect 9: 40 a očekávej 60
  {   L___, B54,  L___, L___, LON,              STRIP_OFF                     },    // Aspect 10: 40 a očekávej 80
  {   L___, LON,  L___, L___, LON,              STRIP_60                      },    // Aspect 11: 60 a volno
  {   LON,  L___, L___, L___, LON,              STRIP_60                      },    // Aspect 12: 60 a výstraha
  {   B54,  L___, L___, L___, LON,              STRIP_60                      },    // Aspect 13: 60 a očekávej 40
  {   B108, L___, L___, L___, LON,              STRIP_60                      },    // Aspect 14: 60 a očekávej 60
  {   L___, B54,  L___, L___, LON,              STRIP_60                      },    // Aspect 15: 60 a očekávej 80
  {   LON,  L___, L___, LON,  LON,              STRIP_OFF                     },    // Aspect 16: 40 a opakovaná výstraha               *
  {   B54,  L___, L___, LON,  LON,              STRIP_OFF                     },    // Aspect 17: 40 a opakovaná očekávej 40            *
  {   B108, L___, L___, LON,  LON,              STRIP_OFF                     },    // Aspect 18: 40 a opakovaná očekávej 60            *
  {   LON,  L___, L___, B54,  L___,             STRIP_OFF                     },    // Aspect 19: Jízda podle rozhledových poměrů       *
  {   LON,  L___, L___, B54,  LON,              STRIP_OFF                     },    // Aspect 20: 40 a jízda podle rozhledových poměrů  *
  {   L___, LON,  L___, LON,  L___,             STRIP_OFF                     },    // Aspect 21: Opakovaná volno
  {   LON,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 22: Opakovaná výstraha
  {   B54,  L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 23: Opakovaná očekávej 40
  {   B108, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 24: Opakovaná očekávej 60
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 25: -----------------------
  {   STRIP_OFF,                                B108, L___, L___, L___, L___  },    // Aspect 26: Jízda vlaku dovolena                  *
  {   L___, L___, L___, L___, L___,             LON,  L___, L___, L___, L___  },    // Aspect 27: Posun zakázán
  {   L___, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 28: Posun dovolen
  {   L___, L___, LON,  LON,  L___,             STRIP_OFF                     },    // Aspect 29: Posun dovolen - nezabezpečený
  {   L___, L___, L___, B54,  L___,             STRIP_OFF                     },    // Aspect 30: Opatrně na přivolávací návěst bez červené
  {   L___, L___, LON,  B54,  L___,             STRIP_OFF                     },    // Aspect 31: Opatrně na přivolávací návěst
};

const SignalSet32 csdMechanicalAspects PROGMEM = {
  {   L___, L___, LON,  L___, L___,             STRIP_OFF                     },    // Aspect 0: Stuj
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 1:  ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 2:  ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 3:  ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 4: ----------------------------           
  {   L___, L___, B22, L___,                    STRIP_OFF                     },    // Aspect 5: Odjezdové návěstidlo dovoluje jízdu    
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 6: ----------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 7: ----------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 8: ----------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 9: ----------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 10: ----------------------------          
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 11: ----------------------------          
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 12: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 13: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 14: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 15: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 16: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 17: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 18: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 19: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 20: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 21: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 22: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 23: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 24: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 25: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 26: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 27: ---------------------------           
  {   L___, L___, L___, LON,  L___,             STRIP_OFF                     },    // Aspect 28: Posun dovolen
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 29: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 30: ---------------------------           
  {   STRIP_OFF,                                STRIP_OFF                     },    // Aspect 31: ---------------------------           
};

void signalMastChangeAspectCsdBasic(int nrSignalMast, byte newAspect) {
  signalMastChangeAspect((int)&(csdBasicAspects[newAspect]), sizeof(csdBasicAspects) / sizeof(csdBasicAspects[0]), nrSignalMast, newAspect);
}

void signalMastChangeAspectCsdIntermediate(int nrSignalMast, byte newAspect) {
  signalMastChangeAspect((int)&(csdIntermediateAspects[newAspect]), sizeof(csdBasicAspects) / sizeof(csdBasicAspects[0]), nrSignalMast, newAspect);
}

void signalMastChangeAspectCsdEmbedded(int nrSignalMast, byte newAspect) {
  signalMastChangeAspect((int)&(csdEmbeddedAspects[newAspect]), sizeof(csdBasicAspects) / sizeof(csdBasicAspects[0]), nrSignalMast, newAspect);
}

void signalMastChangeAspectSzdcBasic(int nrSignalMast, byte newAspect) {
  signalMastChangeAspect((int)&(szdcBasicAspects[newAspect]), sizeof(csdBasicAspects) / sizeof(csdBasicAspects[0]), nrSignalMast, newAspect);
}

void signalMastChangeAspectCsdMechanical(int nrSignalMast, byte newAspect) {
  signalMastChangeAspect((int)&(csdMechanicalAspects[newAspect]), sizeof(csdBasicAspects) / sizeof(csdBasicAspects[0]), nrSignalMast, newAspect);
}

void signalMastChangeAspect(int progMemOffset, int tableSize, int nrSignalMast, byte newAspect) {
  if (debugAspects) {
    Serial.print(F("Change mast ")); Serial.print(nrSignalMast); Serial.print(F(" to aspect ")); Serial.println(newAspect);
  }
  if (nrSignalMast < 0 || nrSignalMast >= NUM_SIGNAL_MAST) {
    Serial.print(F("Invalid mast ")); Serial.println(nrSignalMast);
    return;
  }
  if (newAspect >= tableSize) {
    Serial.print(F("Invalid aspect ")); Serial.println(newAspect);
    return;
  }
  signalMastCurrentAspect[nrSignalMast] = newAspect;

  // Configuration of light outputs
  LightFunction buffer[maxOutputsPerMast];

  // Must copy bytes from PROGMEM to bufferon stack:
  byte* out = (byte*)(void*)&(buffer[0]);
  for (int i = 0; i < sizeof(buffer); i++) {
    *out = pgm_read_byte_near(progMemOffset + i);
    out++;
  }

  // Process LightFunction instructioins
  for (int i = 0; i < maxOutputsPerMast; i++) {
    const oneLightOutputs& outConfig = *(lightConfiguration[i]);
    int nOutput = outConfig[nrSignalMast];
    changeLightState2(nOutput, buffer[i]);
  }
}

void resetStartTime(int lightOutput) {
  unsigned int v = currentTime & 0xffff;
  if (v == 0) {
    v++;
  }
  lightStartTimeBubl[lightOutput] = v;
}

/**********************************************************************************
 *
 */
void processFadeOnOrOff(byte nrOutput, boolean fadeOn) {
  int idx;
  int elapsed = timeElapsedForBulb(nrOutput);

  int limit = (sizeof(fadeTimeLight) / sizeof(fadeTimeLight[0]));

  if (debugFadeOnOff) {
    if (fadeOn) {
      Serial.print("Fade ON, output="); 
    } else {
      Serial.print("Fade OFF, output="); 
    }
    Serial.print(nrOutput);
  }
  if (elapsed > fadeTimeLight[limit - 1]) {
    if (debugFadeOnOff) {
      Serial.println("Reached limit");
    }
    ShiftPWM.SetOne(nrOutput, fadeOn ? maxBrightness : minBrightness);
    return;
  }
  for (idx = 0; idx < limit && elapsed > fadeTimeLight[idx]; idx++) ;
  // idx will be one higher than last less elapsed time, so if elapsed > fadeTimeLight[idx], then becomes 2.
  if (idx >= limit) {
    // should never happen, but prevents out-of-range access
    ShiftPWM.SetOne(nrOutput, fadeOn ? maxBrightness : minBrightness);
    return;
  }
  
  int span = (maxBrightness - minBrightness);
  int pwm = ((span * FADE_COUNTER_LIGHT_1[idx]) / FADE_COUNTER_LIGHT_2[idx]);
  if (fadeOn) {
    pwm += minBrightness;
  } else {
    pwm = maxBrightness - pwm;
  }
  if (debugFadeOnOff) {
    Serial.print(" idx="); Serial.print(idx); Serial.print(", counters: "); 
    Serial.print(FADE_COUNTER_LIGHT_1[idx]); Serial.print("/"); Serial.print(FADE_COUNTER_LIGHT_2[idx]); 
    Serial.print(", pwm = "); Serial.print(pwm);
    Serial.print(", time="); Serial.println(elapsed);
  }
  ShiftPWM.SetOne(nrOutput, pwm);
}

void processFadeOn(byte nrOutput) {
  processFadeOnOrOff(nrOutput, true);
  return;
  int idx = 0;
  int limit = (sizeof(fadeTimeLight) / sizeof(fadeTimeLight[0]));

  int elapsed = timeElapsedForBulb(nrOutput);
  for (idx = 0; idx < limit && elapsed > fadeTimeLight[idx]; idx++) ;
  int span = (maxBrightness - minBrightness);
  int pwm = idx >= limit ? maxBrightness : ((span * FADE_COUNTER_LIGHT_1[idx]) / FADE_COUNTER_LIGHT_2[idx]) + minBrightness;

  ShiftPWM.SetOne(nrOutput, pwm);

  if (debugFadeOnOff) {
    Serial.print("Fade ON, output="); Serial.print(nrOutput); Serial.print("idx="); Serial.print(idx); Serial.print(", counters: "); 
    Serial.print(FADE_COUNTER_LIGHT_1[idx]); Serial.print("/"); Serial.print(FADE_COUNTER_LIGHT_2[idx]); 
    Serial.print(", time="); Serial.print(elapsed);
    Serial.print(" pwm="); Serial.println(pwm);
  }
}

/**********************************************************************************
 *
 */
void processFadeOff(byte nrOutput) {
  processFadeOnOrOff(nrOutput, false);
  return;
  int idx;
  int limit = (sizeof(fadeTimeLight) / sizeof(fadeTimeLight[0]));
  int elapsed = timeElapsedForBulb(nrOutput);
  for (idx = 0; idx < limit && elapsed > fadeTimeLight[idx]; idx++) ;
  
  int span = (maxBrightness - minBrightness);
  int pwm = idx >= limit ? minBrightness : maxBrightness - ((span * FADE_COUNTER_LIGHT_1[idx]) / FADE_COUNTER_LIGHT_2[idx]);

  ShiftPWM.SetOne(nrOutput, pwm);

  if (debugFadeOnOff) {
    Serial.print("Fade OFF, output="); Serial.print(nrOutput); Serial.print("idx="); Serial.print(", counters: "); 
    Serial.print(FADE_COUNTER_LIGHT_1[idx]); Serial.print("/"); Serial.print(FADE_COUNTER_LIGHT_2[idx]); 
    Serial.print(", time="); Serial.print(elapsed);
    Serial.print(" pwm="); Serial.println(pwm);
  }
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
    initializeFadeTime();
    return ;
  }

  if (CV >= START_CV_OUTPUT && CV <= END_CV_OUTPUT) {
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
