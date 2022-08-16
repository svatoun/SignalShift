#include <NmraDcc.h>
#include "Common.h"

boolean handleSignals(ModuleCmd cmd);
ModuleChain signals("signals", 0, &handleSignals);

void commandClear() {}

void commandSetSignal() {
  int nMast = nextNumber();
  if (nMast < 1 || nMast > NUM_SIGNAL_MAST) {
    Serial.println(F("Invalid mast ID"));
    return;
  }
  int mastID = nMast - 1;
  int aspect = nextNumber();
  int maxAspect = 1 << signalMastNumberAddress[mastID];
  if (aspect < 1 || aspect > maxAspect) {
    Serial.println(F("Invalid aspect ID"));
    return;
  }
  signalMastChangeAspect(mastID, aspect - 1);
  Serial.print(F("Mast #")); Serial.print(nMast); Serial.print(F(" set to aspect #")); Serial.println(aspect);
}

void commandGetCV() {
  int cv = nextNumber();
  if (cv < 1 || cv > 512) {
    Serial.println(F("Invalid CV number"));
    return;
  }
  int val = Dcc.getCV(cv);
  Serial.print(F(" CV #")); Serial.print(cv); Serial.print(F(" = ")); Serial.println(val);
}

void commandSetCV() {
  int cv = nextNumber();
  if (cv < 1 || cv > 512) {
    Serial.println(F("Invalid CV number"));
    return;
  }
  int val = nextNumber();
  if (val < 0 || val > 255) {
    Serial.println(F("Invalid CV value"));
  }
  int oldV = Dcc.getCV(cv);
  Dcc.setCV(cv, val);
  Serial.print(F("Changed CV #")); Serial.print(cv); Serial.print(F(": ")); Serial.print(oldV); Serial.print(F(" => ")); Serial.println(val);
}


boolean handleSignals(ModuleCmd cmd) {
  switch (cmd) {
    case initialize:
      registerLineCommand("ASP", &commandSetSignal);
      registerLineCommand("SET", &commandSetCV);
      registerLineCommand("GET", &commandGetCV);
      break;
  }
}