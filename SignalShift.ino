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
const boolean debugLights = true;
const boolean debugAspects = true;

const uint8_t SW_VERSION = 12;

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

const int SEGMENT_SIZE = maxOutputsPerMast + 3;

// --------- HARDWARE PIN WIRING / SETUP --------------------
const byte ACK_BUSY_PIN = A0;

// ShiftPWM HW setup setup
const byte ShiftPWM_dataPin = 11;
const byte ShiftPWM_clockPin = 13;
const byte ShiftPWM_latchPin = 10;

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
const uint16_t CV_AUXILIARY_ACTIVATION = 2;
const uint16_t CV_DECODER_KEY = 15;
const uint16_t CV_DECODER_LOCK = 16;
const uint16_t CV_RESET_TYPE = 33;
const uint16_t CV_ROCO_ADDRESS = 34;
const uint16_t CV_FADE_RATE = 39;
const uint16_t CV_NUM_SIGNAL_NUMBER = 40;
const uint16_t CV_ASPECT_LAG = 41;

const uint16_t CV_PROD_ID_1 = 47;
const uint16_t CV_PROD_ID_2 = 48;
const uint16_t CV_PROD_ID_3 = 49;
const uint16_t CV_PROD_ID_4 = 50;

const uint16_t CV_NUM_MAST_TYPE_START = 51;
const uint16_t CV_NUM_MAST_TYPE_END = CV_NUM_MAST_TYPE_START + NUM_SIGNAL_MAST;

const uint16_t CV_NUM_MAST_PIN_START = 70;
const uint16_t CV_NUM_MAST_PIN_END = CV_NUM_MAST_PIN_START + NUM_SIGNAL_MAST;
static_assert(CV_NUM_MAST_TYPE_END < CV_NUM_MAST_PIN_START, "Too many masts, CV overlap");

const uint16_t START_CV_OUTPUT = 128;
const uint16_t END_CV_OUTPUT = START_CV_OUTPUT + (SEGMENT_SIZE * NUM_SIGNAL_MAST - 1);

const uint16_t START_CV_ASPECT_TABLE = 512;
static_assert(END_CV_OUTPUT <= START_CV_ASPECT_TABLE, "Too many outputs per masts, CV overlap");
static_assert(START_CV_ASPECT_TABLE + NUM_SIGNAL_MAST * maxAspects <= 1024, "Too many mast aspects, out of memory");

/* ------------- Default values for supported CVs ---------- */

const uint8_t VALUE_AUXILIARY_ACTIVATION = 3;  // change this value to restore CV defaults after upload sketch
const uint8_t VALUE_DECODER_KEY = 0;           // unlocked decoder
const uint8_t VALUE_DECODER_LOCK = 0;          // unlocked decoder

const uint8_t VALUE_ROCO_ADDRESS = 1;       // 1 - ROCO address, 0 - LENZ address
const uint8_t VALUE_FADE_RATE = 2;          // 0 - 7
const uint8_t VALUE_NUM_SIGNAL_NUMBER = 8;  // 1 - NUM_SIGNAL_MAST
const uint8_t VALUE_ASPECT_LAG = 1;         // 0 - 255   LAG × 0,128 s

const uint8_t VALUE_PROD_ID_1 = 1;  // productID #1
const uint8_t VALUE_PROD_ID_2 = 1;  // productID #2
const uint8_t VALUE_PROD_ID_3 = 1;  // productID #3
const uint8_t VALUE_PROD_ID_4 = 4;  // productID #4

const int NUM_8BIT_SHIFT_REGISTERS = (NUM_OUTPUTS + 7) / 8;

const int maxAspectBits = 5;
static_assert(maxAspects <= (1 << maxAspectBits), "Too many aspects, do not fit in 5 bits");

const byte ONA = NUM_OUTPUTS;  // OUTPUT_NOT_ASSIGNED

const uint8_t INIT_DECODER_ADDRESS = 100;  // ACCESSORY DECODER ADDRESS default
uint16_t thisDecoderAddress = 100;         // ACCESSORY DECODER ADDRESS
uint16_t maxDecoderAddress = 0;

int signalMastNumberIdx[40];
int posNumber[40];

//   connect                 A   B   C   D   E   F   G   H   I   J   K   L   M   N   O   P
//   offset                  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
//const byte OUTPUT_PIN[] = { 19, 18,  9,  8,  7,  6,  5,  4, 10, 11, 12,  3, 14, 15, 16, 17 };
//   pins                   A5  A4  D9  D8  D7  D6  D5  D4 D10 D11 D12  D3  A0  A1  A2  A3

// Signal sets defined in the decoder
enum SignalSet : byte {
  SIGNAL_SET_CSD_BASIC = 0,         // ČSD basic signal set
  SIGNAL_SET_CSD_INTERMEDIATE = 1,  // ČSD intermediate signal set
  SIGNAL_SET_CSD_EMBEDDED = 2,      // ČSD embedded signal set
  SIGNAL_SET_SZDC_BASIC = 3,        // SŽDC basic signal set
  SIGNAL_SET_CSD_MECHANICAL = 4,    // ČSD mechanical signal set

  _signal_set_last  // must be last
};

const byte maxSignalSets = 16;
const byte maxMastTypes = 16;

enum SignalSetFlags : byte {
  turnoutNoDirection = 0x10,
  // signal controlled by an extended packet
  extendedPacket = 0x20,
  // signal controlled as a series of turnouts
  turnoutControl = 0x40,
  // signal controlled by a binary number encoded into turnout states
  bitwiseControl = 0x60,
  // if set, the signal is rather driven by a codetable (which maps to aspects),
  // rather than the aspect table itself.
  usesCodes  = 0x80,

  signalSetFlags = 0xf0,
  signalSetControlType = (signalSetFlags - usesCodes)
};
static_assert(_signal_set_last <= ((~signalSetFlags) & 0xff), "Too many signal sets");

inline byte toSignalSetIndex(byte b) { 
  byte x = b & (~signalSetFlags);
  return x >= maxSignalSets ? 0 : x;
}

extern const byte mastTypeDefinitionCount;

inline byte toTemplateIndex(byte b) { 
  byte x = b & (~signalSetFlags);
  return x >= mastTypeDefinitionCount ? 0 : x;
}

// Signs on an individual light. Fixed (on, off) and blinking.
enum LightSign : byte {
  inactive = 0x00,  // since initialized globals are zeroed after initializer till end of object
  fixed = 0x01,
  blinking54,
  blinking108,
  blinking45,
  blinking22,

  _lightsign_last  // must be last
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
  static_assert(_lightsign_last <= 16, "Too many signs, must fit in 4 bits (LightFunction::sign)");

  /**
   * True to change the light towards OFF, false to change towards ON.
   */
  boolean off : 1;

  /**
   * If true, the light reached the on/off terminal state
   */
  boolean end : 1;

  // Default initializer: output off(=true), final brightness (end=true), not alternating
  LightFunction()
    : sign(inactive), off(true), end(true) {}

  // Normal construction
  LightFunction(LightSign sign, boolean initOff)
    : sign(sign), off(initOff), end(false) {}

  LightFunction(byte data) {
    *((byte*)(void*)this) = data;
  }

  // Copy constructor for easy assignment; make fast assingment of the whole byte.
  LightFunction(const LightFunction& f) {
    *((byte*)(void*)this) = *((byte*)(void*)&f);
  }
};

const unsigned long BLINKING_TIME_54 = 556;
const unsigned long BLINKING_TIME_108 = 278;
const unsigned long BLINKING_TIME_45 = 667;
const unsigned long BLINKING_TIME_22 = 1333;

// PENDING: The brightness of LED is perceived non-lineraly by human eye. Revise the scale
// here according to https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
const byte FADE_TIME_LIGHT[11] = { 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66 };
const int FADE_COUNTER_LIGHT_1[11] = { /* -1 */ 0, 1, 1, 1, 1, 1, 1, 2, 3, 6, 9 };
const int FADE_COUNTER_LIGHT_2[11] = { /* -1 */ 1, 10, 7, 4, 3, 2, 2, 3, 4, 7, 10 };

// Will be initialized at startup, and whenever fadeRate global changes. As fadeRate is max 7, the value nicely fits in 9 bits unsigned.
unsigned int fadeTimeLight[11] = {};

/**
 * Defines a mast prototype. Each mast uses
 * - a signal set
 * - recognizes a defined number of codes
 * - translates a code into an aspect from the signal set
 */
struct MastTypeDefinition {
  // number of different codes. Implies number of addresses used.
  byte codeCount;
  // default light count
  byte lightCount;
  SignalSet signalSet;
  byte defaultCode;

  byte outputs[maxOutputsPerMast];
  byte code2Aspect[maxAspects];
};

enum MastType {
  none = 0,
  incoming5,
  departure4,
  departure3,
  shutning2,
};

byte signalMastLightYellowUpperOutput[NUM_SIGNAL_MAST] = { 0, 5, 10, 15, 20, 25, ONA, ONA };          // yellow upper
byte signalMastLightGreenOutput[NUM_SIGNAL_MAST] = { 1, 6, 11, 16, 21, 26, ONA, ONA };                // green
byte signalMastLightRedOutput[NUM_SIGNAL_MAST] = { 2, 7, 12, 17, 22, 27, ONA, ONA };                  // red
byte signalMastLightLunarOutput[NUM_SIGNAL_MAST] = { 3, 8, 13, 18, 23, 28, ONA, ONA };                // lunar
byte signalMastLightYellowLowerOutput[NUM_SIGNAL_MAST] = { 4, 9, 14, 19, 24, 29, ONA, ONA };          // yellow lower
byte signalMastLightBlueOutput[NUM_SIGNAL_MAST] = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };         // blue
byte signalMastLightGreenStripOutput[NUM_SIGNAL_MAST] = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };   // green strip
byte signalMastLightYellowStripOutput[NUM_SIGNAL_MAST] = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };  // yellow strip
byte signalMastLightLunarLowerOutput[NUM_SIGNAL_MAST] = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };   // lunar lower
byte signalMastLightBackwardOutput[NUM_SIGNAL_MAST] = { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA };     // backward

typedef byte oneLightOutputs[NUM_SIGNAL_MAST];

oneLightOutputs* lightConfiguration[maxOutputsPerMast] = {
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

SignalSet signalMastSignalSet[NUM_SIGNAL_MAST] = { SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC, SIGNAL_SET_SZDC_BASIC };  // signal set

byte signalMastNumberAddress[NUM_SIGNAL_MAST] = { 1, 1, 1, 1, 1, 1, 1, 1 };  // number of address

byte signalMastCurrentAspect[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };
byte signalMastDefaultAspectIdx[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };

LightFunction bublState2[NUM_OUTPUTS] = {};

unsigned int lightStartTimeBubl[NUM_OUTPUTS] = {};

byte signalMastLastCode[NUM_SIGNAL_MAST] = { 255, 255, 255, 255, 255, 255, 255, 255 };                // last code
unsigned long signalMastLastTime[NUM_SIGNAL_MAST] = {};                                               // last time
boolean signalMastCodeChanged[NUM_SIGNAL_MAST] = { true, true, true, true, true, true, true, true };  // code changed
byte overrides[(NUM_OUTPUTS + 7) / 8] = { false };
byte signalStates[NUM_OUTPUTS] = { 0 };\

boolean reinitializeLocalVariables = true;

int counterNrOutput = 0;
int counterNrSignalMast = 0;
unsigned long currentTime;
unsigned long aspectLag;


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
  Serial.print(F("UNI16ARD-NAV-Shift version "));
  Serial.println(SW_VERSION);
  Serial.println(F("Copyright (c) 2018, Petr Sidlo <sidlo64@seznam.cz>\nCopyright (c) 2022, Svatopluk Dedic <svatopluk.dedic@gmail.com>, APL 2.0 License"));
  Serial.println("Booting...");

  // initialize the digital pins as an outputs
  pinMode(ACK_BUSY_PIN, OUTPUT);
  digitalWrite(ACK_BUSY_PIN, LOW);

  setupShiftPWM();
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(0, 2, 1);

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.initAccessoryDecoder(MAN_ID_DIY, SW_VERSION, FLAGS_OUTPUT_ADDRESS_MODE, 0);

  uint8_t cvVersion = Dcc.getCV(CV_AUXILIARY_ACTIVATION);
  if (cvVersion != VALUE_AUXILIARY_ACTIVATION) {
    setFactoryDefault();
  }

  initLocalVariables();
  ModuleChain::invokeAll(initialize);

  Serial.print(F("Driving max "));
  Serial.print(NUM_OUTPUTS);
  Serial.print(F(" outputs from max "));
  Serial.print(NUM_SIGNAL_MAST);
  Serial.println(F(" masts"));

  initTerminal();
  setupTerminal();
}

void setupShiftPWM() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);

  pinMode(2, INPUT);

  ShiftPWM.SetAmountOfRegisters(NUM_8BIT_SHIFT_REGISTERS);
  ShiftPWM.SetPinGrouping(1);  //This is the default, but I added here to demonstrate how to use the funtion
  ShiftPWM.Start(pwmFrequency, maxBrightness);
  ShiftPWM.SetAll(maxBrightness);
}


/**************************************************************************
 * Main loop.
 */
void loop() {
  maybeInitLocalVariables();
  processTerminal();
  currentTime = millis();

  Dcc.process();

  processAspectCode(counterNrSignalMast);
  counterNrSignalMast++;
  if (counterNrSignalMast >= numSignalNumber) {
    counterNrSignalMast = 0;
  }

  processOutputLight(counterNrOutput);
  counterNrOutput++;
  if (counterNrOutput >= NUM_OUTPUTS) {
    counterNrOutput = 0;
  }
}

/**************************************************************************
 * 
 */
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {

  if (rocoAddress) {
    Addr = Addr + 4;
  }


  if ((Addr < thisDecoderAddress)
      || (Addr >= maxDecoderAddress)) {
    return;
  }

  uint16_t idx = Addr - thisDecoderAddress;

  signalMastChangePos(signalMastNumberIdx[idx], posNumber[idx], Direction);
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

struct MastSettings {
  byte outputs[maxOutputsPerMast];
  byte signalSetOrMastType;
  byte defaultCodeOrAspect;
  byte addresses;
};

const MastSettings factorySignalMastOutputs[] PROGMEM = {
  { { 0, 1, 2, 3, 4, ONA, ONA, ONA, ONA, ONA }, 0x80, 0, 5 },    // signal mast 0
  { { 5, 6, 7, 8, 9, ONA, ONA, ONA, ONA, ONA }, 0x80, 0, 5 },    // signal mast 1
  { { 10, 11, 12, 13, 14, ONA, ONA, ONA, ONA, ONA }, 0x80, 0, 5 },    // signal mast 2
  { { 15, 16, 17, 18, 19, ONA, ONA, ONA, ONA, ONA }, 0x80, 0, 5 },    // signal mast 3
  { { 20, 21, 22, 23, 24, ONA, ONA, ONA, ONA, ONA }, 0x80, 0, 5 },    // signal mast 4
  { { 25, 26, 27, 28, 29, ONA, ONA, ONA, ONA, ONA }, 0x80, 0, 5 },    // signal mast 5
  { { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA }, 0, 0, 5 },    // signal mast 6
  { { ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA }, 0, 0, 5 },    // signal mast 7
  /*
  0, 1, 2, 3, 4, ONA, ONA, ONA, ONA, ONA, 0, 0, 5,            // signal mast 0
  5, 6, 7, 8, 9, ONA, ONA, ONA, ONA, ONA, 0, 0, 5,            // signal mast 1
  10, 11, 12, 13, 14, ONA, ONA, ONA, ONA, ONA, 0, 0, 5,       // signal mast 2
  15, 16, 17, 18, 19, ONA, ONA, ONA, ONA, ONA, 0, 0, 5,       // signal mast 3
  20, 21, 22, 23, 24, ONA, ONA, ONA, ONA, ONA, 0, 0, 5,       // signal mast 4
  25, 26, 27, 28, 29, ONA, ONA, ONA, ONA, ONA, 0, 0, 5,       // signal mast 5
  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0, 0, 5,  // signal mast 6
  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0, 0, 5   // signal mast 7
  */
};

const uint8_t noSignalMastOutputs[] PROGMEM = {
  ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, ONA, 0, 0, 5  // signal mast 7
};

void setFactoryDefault() {
  Serial.println(F("Resetting to factory defaults"));
  uint8_t FactoryDefaultCVSize = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

  for (uint16_t i = 0; i < FactoryDefaultCVSize; i++) {
    Dcc.setCV(FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  }

  int cvNumber = START_CV_OUTPUT;
  int pgmIndex = (int)(void*)factorySignalMastOutputs;
  Serial.println("Setting default mast settings.");
  for (int i = 0; i < sizeof(factorySignalMastOutputs); i++, cvNumber++) {
    Dcc.setCV(cvNumber, pgm_read_byte_near(pgmIndex + i));
  }

  Serial.println("Clearing rest of signal CVs");

  for (int i = sizeof(factorySignalMastOutputs) / SEGMENT_SIZE; i < NUM_SIGNAL_MAST; i++) {
    for (int j = 0; j < sizeof(noSignalMastOutputs); j++, cvNumber++) {
      Dcc.setCV(cvNumber, pgm_read_byte_near(noSignalMastOutputs + j));
    }
  }

  uint16_t OutputCV = START_CV_ASPECT_TABLE;
  for (uint16_t i = 0; i < NUM_SIGNAL_MAST; i++) {
    for (uint16_t j = 1; j <= maxAspects; j++) {
      Dcc.setCV(OutputCV, j);
      OutputCV++;
    }
  }

  // define CVs based on the predefined templates:
  Serial.print(F("pgmIndex base ")); Serial.println((int)factorySignalMastOutputs, HEX);
  Serial.print(F("count ")); Serial.print(sizeof(factorySignalMastOutputs) / sizeof(factorySignalMastOutputs[0]));
  for (int mast = 0; mast < sizeof(factorySignalMastOutputs) / sizeof(factorySignalMastOutputs[0]); mast++) {
    const MastSettings& settingsDef = ((MastSettings*)factorySignalMastOutputs)[mast];
    int cvBase = (sizeof(MastSettings) * mast) + START_CV_OUTPUT;

    Serial.print(F("pgmIndex for mast ")); Serial.print(mast); Serial.print(" = "); Serial.println((int)(&settingsDef), HEX);
    Serial.print(F("cvBase for mast ")); Serial.print(mast); Serial.print(" = "); Serial.println(cvBase);
    int pgmIndex = (int)(&settingsDef.signalSetOrMastType);
    
    byte signalSetOrMastType = pgm_read_byte_near(pgmIndex);
    if ((signalSetOrMastType & usesCodes) > 0) {
      Serial.print(F("Using template for mast ")); Serial.println(mast);

      const struct MastTypeDefinition& def = copySignalMastTypeDefinition(toTemplateIndex(signalSetOrMastType));
      int codeCount = def.codeCount;
      int lightCount = def.lightCount;

      printByteBuffer((byte*)&def, sizeof(def)); Serial.println();
      saveTemplateOutputsToCVs(def, mast);

      pgmIndex = (int)(&settingsDef.defaultCodeOrAspect);
      byte val = pgm_read_byte_near(pgmIndex);
      int cv = cvBase + ((int)(&settingsDef.defaultCodeOrAspect)) - ((int)(&settingsDef));
      if (val != 0xff) {
        val = def.defaultCode;
        Serial.print(F("Setting cv default code ")); Serial.print(cv); Serial.print(" to "); Serial.println(val);
        Dcc.setCV(cv, val);
      }
      pgmIndex++; cv++;
      val = pgm_read_byte_near(pgmIndex);
      if (val == 0 || val > 16) {
        val = findRequiredAddrCount(codeCount, signalSetOrMastType);
      }
      Serial.print(F("Setting cv addresses ")); Serial.print(cv); Serial.print(" to "); Serial.println(val);
      Dcc.setCV(cv, val);

      cv = START_CV_ASPECT_TABLE + mast * maxAspects;
      for (int i = 0; i < sizeof(def.code2Aspect); i++) {
        Dcc.setCV(cv, def.code2Aspect[i]);
        cv++;
      }
    }
  }
  initLocalVariables();
}

void printByteBuffer(const byte* buf, int size) {
  for (int i = 0; i < size; i++) {
    if (i > 0) {
      Serial.print('-');
    }
    int n = *buf;
    buf++;
    if (n < 0x10) {
      Serial.print('0');
    }
    Serial.print(n, HEX);
  }
}

int findRequiredAddrCount(int codes, int mastType) {
  switch (mastType & signalSetControlType) {
    case turnoutNoDirection:
      return codes;
    case extendedPacket:
      return 1;
    case turnoutControl:
      return (codes + 1) / 2;
    case bitwiseControl:
      for (int i = 1; i < 16; i++) {
        int max = (1 << i);
        if (max > codes) {
          return i;
        }
      }
      return 16;
    default:
      ;
  }
}

int findMinLightIndex(int mast) {
  int cvStart = START_CV_OUTPUT + mast * maxOutputsPerMast;
  int min = 255;
  for (int i = cvStart; i < cvStart + maxOutputsPerMast; i++) {
    int x = Dcc.getCV(i);
    if (x == ONA || x == 255) {
      continue;
    }
    if (min > x) {
      min = x;
    }
  }
  return min == 255 ? -1 : min;
}

int findLightCount(int mast) {
  int cnt = 0; 
  int cvStart = START_CV_OUTPUT + mast * maxOutputsPerMast;
  for (int i = cvStart; i < cvStart + maxOutputsPerMast; i++) {
    int x = Dcc.getCV(i);
    if (x == ONA || x == 255) {
      continue;
    }
    cnt++;
  }
  return cnt;
}

int findNextLight(int mast) {
  if (mast == 0) {
    return 0;
  }

  int cvStart = START_CV_OUTPUT + (mast - 1) * maxOutputsPerMast;
  int max = -1;
  for (int i = cvStart; i < cvStart + maxOutputsPerMast; i++) {
    int x = Dcc.getCV(i);
    if (x == ONA || x == 255) {
      continue;
    }
    if (max < x) {
      max = x;
    }
  }
  return max + 1;
}

void saveTemplateOutputsToCVs(const struct MastTypeDefinition& def, int mastIndex) {
  Serial.print(F("Setting outputs to mast #")); Serial.print(mastIndex); Serial.print(F(" from def ")); Serial.println((int)(int*)(&def), HEX);
  int minLightNumber = findMinLightIndex(mastIndex);
  int oldLightCount = findLightCount(mastIndex);
  int from = findNextLight(mastIndex);

  int lc = 0;
  int cvIndex = (sizeof(MastSettings) * mastIndex) + START_CV_OUTPUT;
  for (int i = 0; i < sizeof(def.outputs); i++) {
    byte n = def.outputs[i];
    if (n != 0 && n != 255) {
      n += from;
      n--;
      lc++;
    } else {
      n = ONA;
    }
    Serial.print(F("Set CV ")); Serial.print(cvIndex); Serial.print(" := "); Serial.println(n);
    Dcc.setCV(cvIndex, n);
    cvIndex++;
  }
  // skip the aspect table/template.
  cvIndex++;
  Dcc.setCV(cvIndex, def.defaultCode);
}

int reassignMastOutputs(int mastIndex, int from, boolean propagate) {
  int cvIndex = (sizeof(MastSettings) * mastIndex) + START_CV_OUTPUT;
  int minLightNumber = findMinLightIndex(mastIndex);

  int lastOutput = -1;

  Serial.print(F("Setting outputs for ")); Serial.print(mastIndex); Serial.print(F(" start from ")); Serial.println(from);

  for (int i = 0; i < sizeof(MastSettings::outputs); i++) {
    byte n = Dcc.getCV(cvIndex);
    if (n != 255 && n != ONA) {
      n = (n - minLightNumber) + from;
      Dcc.setCV(cvIndex, n);
      lastOutput = n;
    }
    cvIndex++;
  }
  lastOutput++;
  if (!propagate) {
    return lastOutput;
  }
  int nextInput = lastOutput;
  for (int nextMast = mastIndex + 1; nextMast = NUM_SIGNAL_MAST; nextMast++) {
    int min = findMinLightIndex(nextMast);
    if (min >= nextInput) {
      return -1;
    }
    int nextInput = reassignMastOutputs(nextMast, nextInput, false);
    if (nextInput == -1) {
      break;
    }
  }
  return lastOutput;
}

/**********************************************************************************
 * Init local variables.
 */
void initLocalVariables() {
  reinitializeLocalVariables = true;
}

void maybeInitLocalVariables() {
  if (!reinitializeLocalVariables) {
    return;
  }
  reinitializeLocalVariables = false;
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
    signalMastLightYellowUpperOutput[i] = Dcc.getCV(counter);  // yellow upper
    counter++;
    signalMastLightGreenOutput[i] = Dcc.getCV(counter);  // green
    counter++;
    signalMastLightRedOutput[i] = Dcc.getCV(counter);  // red
    counter++;
    signalMastLightLunarOutput[i] = Dcc.getCV(counter);  // lunar
    counter++;
    signalMastLightYellowLowerOutput[i] = Dcc.getCV(counter);  // yellow lower
    counter++;
    signalMastLightBlueOutput[i] = Dcc.getCV(counter);  // blue
    counter++;
    signalMastLightGreenStripOutput[i] = Dcc.getCV(counter);  // green strip
    counter++;
    signalMastLightYellowStripOutput[i] = Dcc.getCV(counter);  // yellow strip
    counter++;
    signalMastLightLunarLowerOutput[i] = Dcc.getCV(counter);  // lunar lower
    counter++;
    signalMastLightBackwardOutput[i] = Dcc.getCV(counter);  // backward
    counter++;

    byte mastTypeOrSignalSet = Dcc.getCV(counter);
    counter++;
    byte signalSet = 0;

    if ((mastTypeOrSignalSet & usesCodes) > 0) {
      signalSet = findMastTypeSignalSet(mastTypeOrSignalSet);
    }
    signalSet = signalSet & (~signalSetFlags);
    if ((signalSet < 0) || (signalSet >= maxSignalSets)) {
      signalSet = 0;
    }

    signalMastSignalSet[i] = signalSet;

    byte defaultAspectIdx = Dcc.getCV(counter);
    counter++;

    signalMastDefaultAspectIdx[i] = defaultAspectIdx;
    signalMastNumberAddress[i] = Dcc.getCV(counter);
    Serial.print(F("Addresses: ")); Serial.println(signalMastNumberAddress[i]);
    counter++;
  }

  maxDecoderAddress = thisDecoderAddress;
  for (int i = 0; i < numSignalNumber; i++) {
    maxDecoderAddress = maxDecoderAddress + signalMastNumberAddress[i];
  }

  int idx = 0;
  for (int i = 0; i < numSignalNumber; i++) {
    int addrs = signalMastNumberAddress[i];
    if (addrs > maxAspectBits) {
      Serial.print(F("Mast #"));
      Serial.print(i);
      Serial.print(F(" Invalid number of addresses: "));
      Serial.println(addrs);
      addrs = 1;
    }
    for (int j = 0; j < addrs; j++) {
      signalMastNumberIdx[idx] = i;
      posNumber[idx] = j;
      signalMastChangePos(signalMastNumberIdx[idx], posNumber[idx], 1);
      signalMastChangePos(signalMastNumberIdx[idx], posNumber[idx], 0);

      idx++;
    }
  }

  for (int i = 0; i < numSignalNumber; i++) {
    for (int j = 0; j < 8; j++) {

      int dir = bitRead(signalMastDefaultAspectIdx[i], j);

      signalMastChangePos(i, j, dir);
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
    Serial.print(F("Change light #"));
    Serial.print(lightOutput);
    Serial.print(F(" curState: "));
    Serial.print(bs.sign);
    Serial.print(F(", off = "));
    Serial.print(bs.off);
    Serial.print(F(", newstate: "));
    Serial.print(newState.sign);
    Serial.print(F(", off = "));
    Serial.println(newState.off);
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
    Serial.print(F("Change light #"));
    Serial.print(lightOutput);
    Serial.print(F(" newState: "));
    Serial.print(bublState2[lightOutput].sign);
    Serial.print(F(", off = "));
    Serial.println(bublState2[lightOutput].off);
  }
}

/**
 * The 74HC595 on the PCB is placed so that its outputs are mirrored: MSB is at the edge, while LSB is in the middle of the
 * pin line. We need to reverse output position within each 8-bit sequence
 */
inline byte numberToPhysOutput(byte nrOutput) {
  return (nrOutput & (~0x07)) + (7 - (nrOutput & 0x07));
}

void setPWM(byte nrOutput, byte level) {
  boolean b = bitRead(overrides[nrOutput / 8], nrOutput & 0x07);
  if (b) {
    return;
  }
  ShiftPWM.SetOne(numberToPhysOutput(nrOutput), level);
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
        Serial.print("Light elapsed ");
        Serial.print(nrOutput);
        Serial.print(F(" off = "));
        Serial.println(off);
      }
      bublState2[nrOutput].end = true;
      setPWM(nrOutput, off ? minBrightness : maxBrightness);
    }

    if (blinkDelay > 0) {
      if (elapsed > blinkDelay) {
        if (debugLightFlip) {
          Serial.print("Light blinked ");
          Serial.println(nrOutput);
        }
        bublState2[nrOutput].end = false;
        bublState2[nrOutput].off = !off;
        resetStartTime(nrOutput);
      }
    } else {
      // disable state changes.
      if (debugLightFlip) {
        Serial.print("Light fixed ");
        Serial.print(nrOutput);
        Serial.print(F(" state "));
        Serial.println(!off);
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
    Serial.print(F("Error: mastChangePos mast out of range: "));
    Serial.println(nrSignalMast);
    return;
  }
  if (pos >= maxOutputsPerMast) {
    Serial.print(F("Error: mastChangePos pos out of range"));
    Serial.println(pos);
    return;
  }
  byte newAspectIdx = signalMastLastCode[nrSignalMast];

  bitWrite(newAspectIdx, pos, Direction);

  switch (signalMastNumberAddress[nrSignalMast]) {
    case 1:
      newAspectIdx = newAspectIdx & 0x01;
      break;
    case 2:
      newAspectIdx = newAspectIdx & 0x03;
      break;
    case 3:
      newAspectIdx = newAspectIdx & 0x07;
      break;
    case 4:
      newAspectIdx = newAspectIdx & 0x0F;
      break;
    default:
      newAspectIdx = newAspectIdx & 0x1F;
      break;
  }

  if (signalMastLastCode[nrSignalMast] == newAspectIdx) {
    return;
  }

  if (newAspectIdx >= maxAspects) {
    Serial.print(F("Error: newAspectIdx out of range"));
    Serial.println(newAspectIdx);
    return;
  }

  signalMastLastCode[nrSignalMast] = newAspectIdx;
  signalMastLastTime[nrSignalMast] = currentTime;
  signalMastCodeChanged[nrSignalMast] = true;
}

/**********************************************************************************
 *
 */
void processAspectCode(int nrSignalMast) {

  if (!signalMastCodeChanged[nrSignalMast]) {
    return;
  }

  unsigned long aspectCodeElapsedTime = currentTime - signalMastLastTime[nrSignalMast];

  if (aspectCodeElapsedTime < aspectLag) {
    return;
  }

  byte newCode = signalMastLastCode[nrSignalMast];
  byte newAspect = aspectJmri(nrSignalMast, newCode);
  signalMastChangeAspect(nrSignalMast, newAspect);
  signalMastCodeChanged[nrSignalMast] = false;
}

/**********************************************************************************
 *
 */
void signalMastChangeAspect(int nrSignalMast, byte newAspect) {
  if (newAspect == 255) {
    signalMastChangeAspectCsdBasic(nrSignalMast, 26);
    return;
  }
  if (newAspect > maxAspects || newAspect == 0) {
    // all off
    signalMastChangeAspectCsdMechanical(nrSignalMast, 1);
    return;
  }
  newAspect--;
  if (signalMastCurrentAspect[nrSignalMast] == newAspect) {
    return;
  }
  switch (signalMastSignalSet[nrSignalMast]) {
    case SIGNAL_SET_CSD_BASIC:
      signalMastChangeAspectCsdBasic(nrSignalMast, newAspect);
      break;
    case SIGNAL_SET_CSD_INTERMEDIATE:
      signalMastChangeAspectCsdIntermediate(nrSignalMast, newAspect);
      break;
    case SIGNAL_SET_CSD_EMBEDDED:
      signalMastChangeAspectCsdEmbedded(nrSignalMast, newAspect);
      break;
    case SIGNAL_SET_SZDC_BASIC:
      signalMastChangeAspectSzdcBasic(nrSignalMast, newAspect);
      break;
    case SIGNAL_SET_CSD_MECHANICAL:
      signalMastChangeAspectCsdMechanical(nrSignalMast, newAspect);
      break;
  }
}

/**********************************************************************************
 *
 */
void signalMastChangeAspect(int progMemOffset, int tableSize, int nrSignalMast, byte newAspect) {
  if (debugAspects) {
    Serial.print(F("Change mast "));
    Serial.print(nrSignalMast);
    Serial.print(F(" to aspect "));
    Serial.println(newAspect);
  }
  if (nrSignalMast < 0 || nrSignalMast >= NUM_SIGNAL_MAST) {
    Serial.print(F("Invalid mast "));
    Serial.println(nrSignalMast);
    return;
  }
  if (newAspect >= tableSize) {
    Serial.print(F("Invalid aspect "));
    Serial.println(newAspect);
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
    setPWM(nrOutput, fadeOn ? maxBrightness : minBrightness);
    return;
  }
  for (idx = 0; idx < limit && elapsed > fadeTimeLight[idx]; idx++)
    ;
  // idx will be one higher than last less elapsed time, so if elapsed > fadeTimeLight[idx], then becomes 2.
  if (idx >= limit) {
    // should never happen, but prevents out-of-range access
    setPWM(nrOutput, fadeOn ? maxBrightness : minBrightness);
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
    Serial.print(" idx=");
    Serial.print(idx);
    Serial.print(", counters: ");
    Serial.print(FADE_COUNTER_LIGHT_1[idx]);
    Serial.print("/");
    Serial.print(FADE_COUNTER_LIGHT_2[idx]);
    Serial.print(", pwm = ");
    Serial.print(pwm);
    Serial.print(", time=");
    Serial.println(elapsed);
  }
  setPWM(nrOutput, pwm);
}

void processFadeOn(byte nrOutput) {
  processFadeOnOrOff(nrOutput, true);
  return;
  int idx = 0;
  int limit = (sizeof(fadeTimeLight) / sizeof(fadeTimeLight[0]));

  int elapsed = timeElapsedForBulb(nrOutput);
  for (idx = 0; idx < limit && elapsed > fadeTimeLight[idx]; idx++)
    ;
  int span = (maxBrightness - minBrightness);
  int pwm = idx >= limit ? maxBrightness : ((span * FADE_COUNTER_LIGHT_1[idx]) / FADE_COUNTER_LIGHT_2[idx]) + minBrightness;

  setPWM(nrOutput, pwm);

  if (debugFadeOnOff) {
    Serial.print("Fade ON, output=");
    Serial.print(nrOutput);
    Serial.print("idx=");
    Serial.print(idx);
    Serial.print(", counters: ");
    Serial.print(FADE_COUNTER_LIGHT_1[idx]);
    Serial.print("/");
    Serial.print(FADE_COUNTER_LIGHT_2[idx]);
    Serial.print(", time=");
    Serial.print(elapsed);
    Serial.print(" pwm=");
    Serial.println(pwm);
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
  for (idx = 0; idx < limit && elapsed > fadeTimeLight[idx]; idx++)
    ;

  int span = (maxBrightness - minBrightness);
  int pwm = idx >= limit ? minBrightness : maxBrightness - ((span * FADE_COUNTER_LIGHT_1[idx]) / FADE_COUNTER_LIGHT_2[idx]);

  setPWM(nrOutput, pwm);

  if (debugFadeOnOff) {
    Serial.print("Fade OFF, output=");
    Serial.print(nrOutput);
    Serial.print("idx=");
    Serial.print(", counters: ");
    Serial.print(FADE_COUNTER_LIGHT_1[idx]);
    Serial.print("/");
    Serial.print(FADE_COUNTER_LIGHT_2[idx]);
    Serial.print(", time=");
    Serial.print(elapsed);
    Serial.print(" pwm=");
    Serial.println(pwm);
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
  Serial.print(F("CV changed: "));
  Serial.print(CV);
  Serial.print(F(" := "));
  Serial.println(Value);
  if (CV == CV_ACCESSORY_DECODER_ADDRESS_LSB) {
    lsb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return;
  }

  if (CV == CV_ACCESSORY_DECODER_ADDRESS_MSB) {
    msb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return;
  }

  if (CV == CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB) {
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, Value);
    lsb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return;
  }

  if (CV == CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB) {
    Value = Value & 0x07;
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, Value);
    msb = Value;
    thisDecoderAddress = (msb << 8) | lsb;
    return;
  }

  if (CV == CV_ROCO_ADDRESS) {
    rocoAddress = Value;
    return;
  }

  if (CV == CV_NUM_SIGNAL_NUMBER) {
    numSignalNumber = Value;
    return;
  }

  if (CV == aspectLag) {
    aspectLag = Value;
    aspectLag = aspectLag << 7;
    return;
  }

  if (CV == CV_DECODER_KEY) {
    decoderKey = Value;
    return;
  }

  if (CV == CV_DECODER_LOCK) {
    decoderLock = Value;
    return;
  }

  if (CV == CV_FADE_RATE) {
    fadeRate = Value;
    initializeFadeTime();
    return;
  }

  if (CV >= START_CV_OUTPUT && CV <= END_CV_OUTPUT) {
    initLocalVariablesSignalMast();
    return;
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

  byte aspectJmri;

  uint16_t cvAdr = START_CV_ASPECT_TABLE;

  cvAdr = cvAdr + maxAspects * nrSignalMast + aspectMx;

  aspectJmri = Dcc.getCV(cvAdr);

  return aspectJmri;
}

extern const MastTypeDefinition mastTypeDefinitions[];

const struct MastTypeDefinition& copySignalMastTypeDefinition(byte typeId) {

  typeId &= (~signalSetFlags);
  if (typeId >= mastTypeDefinitionCount) {
    typeId = 0;
  }
  Serial.print("Copy from id "); Serial.println(typeId);
  static MastTypeDefinition buffer;
  int progMemOffset = mastTypeDefinitions + typeId;
  byte* out = (byte*)(void*)&(buffer);
  for (int i = 0; i < sizeof(buffer); i++) {
    *out = pgm_read_byte_near(progMemOffset + i);
    out++;
  }
  return buffer;
}

byte findMastTypeSignalSet(byte typeId) {
  typeId &= (~signalSetFlags);
  if (typeId >= mastTypeDefinitionCount) {
    typeId = 0;
  }

  int progMemOffset = &mastTypeDefinitions[typeId].signalSet;
  byte signalSet = pgm_read_byte_near(progMemOffset);
  if (signalSet >= _signal_set_last) {
    signalSet = 0;
  }
  return signalSet;
}
