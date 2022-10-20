#include <NmraDcc.h>
#include "Common.h"

boolean handleSignals(ModuleCmd cmd);
ModuleChain signals("signals", 0, &handleSignals);

extern void commandReset();

int definedMast = -1;

void commandClear() {
  setFactoryDefault();
  commandReset();
}

void commandPrintMastDef() {
  int nMast = nextNumber();
  if (nMast < 1 || nMast > NUM_SIGNAL_MAST) {
    Serial.println(F("Invalid mast ID"));
    return;
  }
  printMastDef(nMast - 1);
}

void commandDump() {
  dumpAllMasts();
}

void dumpAllMasts() {
  for (int m = 0; m < NUM_SIGNAL_MAST; m++) {
    if (!isMastActive(m)) {
      continue;
    }
    printMastDef(m);
  }
}

boolean isMastActive(int nMast) {
  for (int i = 0; i < maxOutputsPerMast; i++) {
    const oneLightOutputs& outConfig = *(lightConfiguration[i]);
    int o = outConfig[nMast];
    if ((o != ONA) && (o != 0xff)) {
      return true;
    }
  }
  return false;
}

void printMastDef(int nMast) {
  int first = ONA;
  int cnt = 0;
  for (int i = 0; i < maxOutputsPerMast; i++) {
    const oneLightOutputs& outConfig = *(lightConfiguration[i]);
    int o = outConfig[nMast];
    if ((o != ONA) && (o != 0xff)) {
      if (o < first) {
        first = o;
      }
      cnt++;
    }
  }
  if (first >= ONA) {
    Serial.print(F("DEL:")); Serial.println(nMast + 1);
    return;
  }
  Serial.print(F("DEF:")); Serial.print(nMast + 1); Serial.print(':'); 
  Serial.print(first + 1); Serial.print(':'); Serial.print(cnt); Serial.print(':'); Serial.println(1 << signalMastNumberAddress[nMast]);
  if (signalMastSignalSet[nMast] != 0 || signalMastDefaultAspectIdx[nMast] != 0) {
    Serial.print(F("  SET:")); Serial.print(signalMastSignalSet[nMast]); Serial.print(':'); Serial.println(signalMastDefaultAspectIdx[nMast]);
  }
  Serial.print(F("  OUT:"));
  printMastOutputs(nMast, false);
  printAspectMap(nMast);
  Serial.println(F("END"));
}

void printMastOutputs(int nMast, boolean suppressFull) {
  byte lights[maxOutputsPerMast];
  int cnt = 0;
  for (int i = 0; i < maxOutputsPerMast; i++) {
    const oneLightOutputs& outConfig = *(lightConfiguration[i]);
    lights[i] = outConfig[nMast];
    if (lights[i] < ONA) {
      cnt++;
    }
  }
  int skipCnt = 0;
  int cnt2 =  0;
  for (int x = 0; x < maxOutputsPerMast; x++) {
    int a = lights[x];
    if (a >= ONA) {
      if (skipCnt++ == 0) {
        Serial.print('-');
      }
      continue;
    }
    if (skipCnt > 0) {
      if (skipCnt == 2) {
        Serial.print('-');
      } else if (skipCnt > 2) {
        Serial.print(skipCnt - 1);
      }
      Serial.print(':');
    }
    skipCnt = 0;
    int seq = x + 1;
    while ((seq < sizeof(lights)) && (lights[seq] == (1 + lights[seq - 1]))) {
      seq++;
    }
    int len = seq - x;
    if (x == 0 && len == cnt && suppressFull) {
      return;
    }
    if (len > 2) {
      Serial.print(a + 1); Serial.print('-'); Serial.print(a + len);
      x = seq - 1; // will increment to seq in loop reinit
    } else if (len == 2) {
      Serial.print(a + 1); Serial.print(':'); Serial.print(a + 2);
      x++;
    } else {
      Serial.print(a + 1); 
    }
    cnt2 += len;
    if (cnt2 == cnt) {
      break;
    }
    Serial.print(':');
  }
  Serial.println();
}

int findSameAspect(int nMast) {
  byte aspectMap[maxAspects];

  boolean identity = true;
  int cv = START_CV_ASPECT_TABLE + (nMast * maxAspects);
  for (int x = 0; x < maxAspects; x++, cv++) {
    aspectMap[x] = Dcc.getCV(cv);
    if (aspectMap[x] != x) {
      identity = false;
    }
  }
  if (identity) {
    return NUM_SIGNAL_MAST + 1;
  }
  for (int i = 0; i < nMast; i++) {
    if (signalMastNumberAddress[i] != signalMastNumberAddress[nMast]) {
      continue;
    }
    cv = START_CV_ASPECT_TABLE + (i * maxAspects);
    boolean found = true;
    for (int x = 0; x < maxAspects; x++, cv++) {
      if (Dcc.getCV(cv) != aspectMap[x]) {
        found = false;
        break;
      }
    }
    if (found) {
      return i;
    }
  }
  return -1;
}

void printAspectMap(int nMast) {
  int limit = 1 << signalMastNumberAddress[nMast];
  int copyOf = findSameAspect(nMast);
  if (copyOf >= 0) {
    if (copyOf >= NUM_SIGNAL_MAST) {
      return;
    }
    Serial.print(F("  MCP:")); Serial.println(copyOf + 1);
  } else {
    byte aspectMap[maxAspects];
    int cnt = 0;
    int cv = START_CV_ASPECT_TABLE + (nMast * maxAspects);
    for (int x = 0; x < limit; x++, cv++) {
      aspectMap[x] = Dcc.getCV(cv);
      if (aspectMap[x] < maxAspects) {
        cnt++;
      }
    }
    int skipCnt = 0;
    Serial.print(F("  MAP:"));
    for (int x = 0; x < limit; x++) {
      int a = aspectMap[x];
      int seq = x + 1;
      if (a >= maxAspects) {
        skipCnt++;
        continue;
      }
      if (skipCnt > 0) {
        Serial.print('-');
        if (skipCnt == 2) {
          Serial.print('-');
        } else if (skipCnt > 2) {
          Serial.print(skipCnt);
        }
      }
      if (x > 0) {
        Serial.print(':');
      }
      while ((seq < limit) && (aspectMap[seq] == (1 + aspectMap[seq - 1]))) {
        seq++;
      }
      int len = seq - x;
      if (len > 2) {
        Serial.print(a); Serial.print('-'); Serial.print(a + len - 1);
        x = seq - 1; // will increment to seq in loop reinit
      } else if (len == 2) {
        Serial.print(a); Serial.print(':'); Serial.print(a + 1);
        x = seq - 1; // will increment to seq in loop reinit
      } else {
        Serial.print(a);
      }
    }
    if (skipCnt > 0) {
      Serial.print('-');
      if (skipCnt == 2) {
        Serial.print('-');
      } else if (skipCnt > 2) {
        Serial.print(skipCnt);
      }
    }
  }
  Serial.println();
}

void commandEnd() {
  if (definedMast < 0) {
    Serial.println(F("No open definition."));
    return;
  } else {
    Serial.print(F("Mast #")); Serial.print(definedMast); Serial.println(F(" definition closed."));
    definedMast = -1;
  }
}

/*
  Syntax: DEF:mast:first-out:lights:codes
*/
void commandDefineMast() {
  int nMast = nextNumber();
  if (nMast < 1 || nMast > NUM_SIGNAL_MAST) {
    Serial.println(F("Invalid mast ID"));
    return;
  }

  definedMast = nMast;
  int firstOut = nextNumber();
  if (firstOut == -2) {
    Serial.print(F("Mast #")); Serial.print(nMast); Serial.println(F(" definition open."));
    return;
  }
  if (firstOut < 1 || firstOut > NUM_OUTPUTS) {
    Serial.println(F("Invalid output ID"));
    return;
  }

  int numLights = nextNumber();
  if (numLights < 1 || (firstOut + numLights > NUM_OUTPUTS) || numLights > maxOutputsPerMast) {
    Serial.println(F("Invalid number of lights"));
    return;
  }

  int numSignals = nextNumber();
  if (numSignals == -2) {
    numSignals = 1 << numLights;
  }
  if (numSignals < 1 || numSignals > 32) {
    Serial.println(F("Invalid number of signals"));
    return;
  }
  int check = 2;
  int bits = 1;
  while (check < numSignals) {
    check = check << 1;
    bits++;
  }

  if (bits > 5) {
    Serial.println(F("Invalid number of signals"));
    return;
  }

  int cvBase = START_CV_OUTPUT + (nMast - 1) * SEGMENT_SIZE;

  for (int i = 0; i < maxOutputsPerMast; i++) {
    if (i >= numLights) {
      Dcc.setCV(cvBase + i, ONA);
    } else {
      Dcc.setCV(cvBase + i, firstOut + i - 1);
    }
  }

  // the signal set number
  Dcc.setCV(cvBase + 10, SIGNAL_SET_CSD_BASIC);
  // the default signal
  Dcc.setCV(cvBase + 11, 0);
  // number of addresses = bits
  Serial.print("Wrote to CV #"); Serial.println(cvBase + 12, HEX);
  Dcc.setCV(cvBase + 12, bits);

  signalMastSignalSet[nMast -1] = SIGNAL_SET_CSD_BASIC;
  signalMastDefaultAspectIdx[nMast - 1] = 0;
  signalMastNumberAddress[nMast - 1] = bits;
  Serial.print(F("Mast #")); Serial.print(nMast); Serial.print(F(" uses outputs ")); Serial.print(firstOut); Serial.print(F(" - ")); Serial.print(firstOut + numLights);
  Serial.print(F(" and ")); Serial.print(bits); Serial.println(F(" addresses."));

  Serial.print(F("Definition open, END to finish."));
}

void commandSetSignal() {
  int nMast = nextNumber();
  if (nMast < 1 || nMast > NUM_SIGNAL_MAST) {
    Serial.println(F("Invalid mast ID"));
    return;
  }
  int mastID = nMast - 1;
  int aspect = nextNumber();
  int maxAspect = 1 << signalMastNumberAddress[mastID];
  byte newAspect;

  if (aspect < 1 || aspect > maxAspect) {
    Serial.println(F("Undefined aspect ID"));
    newAspect = 255;
  } else {
    newAspect = aspectJmri(mastID, aspect - 1) ;
  }
  signalMastChangeAspect(mastID, newAspect);
  Serial.print(F("Mast #")); Serial.print(nMast); Serial.print(F(" set to aspect index #")); Serial.print(aspect); Serial.print('='); Serial.println(newAspect);
}

void commandGetCV() {
  int cv = nextNumber();
  if (cv < 1 || cv > 1023) {
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

void commandMastSet() {
  if (definedMast < 0) {
    Serial.println(F("No mast open. Use DEF:"));
    return;
  }
  int nMast = definedMast;
  int signalSet = nextNumber();
  if (signalSet < 0 || signalSet >= _signal_set_last) {
    Serial.println(F("Invalid signal set"));
    return;
  }
  nMast--;
  int defaultAspect = nextNumber();
  int cv = START_CV_OUTPUT + nMast * SEGMENT_SIZE;
  int bits = Dcc.getCV(cv + 12);
  if (defaultAspect < 0 || defaultAspect >= maxAspects || defaultAspect >= (1 << bits)) {
    Serial.println(F("Invalid default aspect"));
    return;
  }
  Dcc.setCV(cv + 10, signalSet);
  Dcc.setCV(cv + 11, defaultAspect);
  signalMastSignalSet[nMast] = signalSet;
  signalMastDefaultAspectIdx[nMast] = defaultAspect;
}

extern int inputDelim;

void commandMapOutput() {
  int nMast = nextNumber();
  if (nMast < 1 || nMast > NUM_SIGNAL_MAST) {
    Serial.println(F("Invalid mast ID"));
    return;
  }
  int out = -1;

  nMast--;
  int outCV = START_CV_OUTPUT + (nMast) * SEGMENT_SIZE;
  int cnt = 0;
  while ((out = nextNumber(true)) != -2) {
    if (out < 1 || out > NUM_OUTPUTS) {
      Serial.println(F("Invalid output"));
      return;
    }
    if (cnt >= maxOutputsPerMast) {
      Serial.println(F("Many outputs"));
      return;
    }
    out--;
    if (inputDelim == '-') {
      int to = nextNumber(false);

      if (to <= out || to > NUM_OUTPUTS) {
        Serial.println(F("Invalid output"));
        return;
      }
      if (to - out > maxOutputsPerMast) {
        Serial.println(F("Many outputs"));
        return;
      }
      Serial.print("out = "); Serial.print(out); Serial.print(" to "); Serial.println(to);
      for (int i = out; i < to; i++) {
        Serial.print("Add output "); Serial.println(out);
        Dcc.setCV(outCV, out);
        oneLightOutputs& outConfig = *(lightConfiguration[cnt]);
        outConfig[nMast] = out;
        cnt++;
        out++;
        outCV++;
      }
    } else {
      Serial.print("Add output "); Serial.println(out);
      Dcc.setCV(outCV, out);
      oneLightOutputs& outConfig = *(lightConfiguration[cnt]);
      outConfig[nMast] = out;
      cnt++;
      outCV++;
    }
  }
  while (cnt < maxOutputsPerMast) {
      Dcc.setCV(outCV, ONA);
      oneLightOutputs& outConfig = *(lightConfiguration[cnt]);
      outConfig[nMast] = ONA;
      cnt++;
  }
}

void commandMapAspects() {
  if (definedMast < 0) {
    Serial.println(F("No mast opened"));
    return;
  }
  int nMast = definedMast - 1;
  int cur = 0;
  int limit = 1 << signalMastNumberAddress[nMast];
  int cvBase = START_CV_ASPECT_TABLE + nMast * maxAspects;
  while (*inputPos) {
    if (*inputPos == ':') {
      inputPos++;
      continue;
    }
    if (cur >= limit) {
      Serial.println(F("Too many aspects"));
      return;
    }
    if (*inputPos == '-') {
      inputPos++;
      if (*inputPos == '-' || *inputPos == ':' || *inputPos == 0) {
        do {
          Dcc.setCV(cvBase + cur, 255);
        } while (*(inputPos++) == '-');
        continue;
      }
      int n = nextNumber();
      if (n < 1 || cur + n > limit) {
        Serial.print(F("Invalid len"));
        return;
      }
      for (int i = 0; i < n; i++) {
        Dcc.setCV(cvBase + cur, 255);
        cur++;
      }
      continue;
    }
    if (*inputPos == '=') {
      inputPos++;
      cur = nextNumber();
      if (cur < 0) {
        Serial.print(F("Invalid index"));
        return;
      }
      continue;
    }

    if (*inputPos == '-') {
      inputPos++;
      cur++;
      continue;
    }
    int aspect = nextNumber(true);
    if (aspect < 0) {
      break;
    }
    if (aspect >= maxAspects) {
      Serial.print(F("Bad aspect"));
    }
    if (inputDelim == '-') {
      int aspectTo = nextNumber();
      if (aspectTo < aspect || aspectTo >= maxAspects) {
        Serial.println(F("Bad range"));
        return;
      }
      for (int x = aspect; x <= aspectTo; x++) {
        Dcc.setCV(cvBase + cur, x);
        cur++;
      }
    } else {
      Dcc.setCV(cvBase + cur, aspect);
      cur++;
    }
  }
}

void commandOverride() {
  int n = nextNumber(true);
  if (n < 1 || n > NUM_OUTPUTS) {
    Serial.println(F("Invalid output"));
    return;
  }
  int to = n;
  if (inputDelim == '-') {
    to = nextNumber();
    if (to < n || to > NUM_OUTPUTS) {
      Serial.println(F("Bad range"));
      return;
    }
  }
  int level = nextNumber();
  if (level > 255) {
      Serial.println(F("Bad pwm"));
      return;
  }
  for (int i = n - 1; i < to; i++) {
    if (level < 0) {
      bitWrite(overrides[i / 8], i & 0x07, false);
      ShiftPWM.SetOne(numberToPhysOutput(i), 0);
    } else {
      bitWrite(overrides[i / 8], i & 0x07, true);
      ShiftPWM.SetOne(numberToPhysOutput(i), level);
    }
  }
}

void commandInf() {

}

boolean handleSignals(ModuleCmd cmd) {
  switch (cmd) {
    case initialize:
      registerLineCommand("SGN", &commandSetSignal);
      registerLineCommand("SET", &commandSetCV);
      registerLineCommand("GET", &commandGetCV);
      registerLineCommand("DEF", &commandDefineMast);
      registerLineCommand("INF", &commandPrintMastDef);
      registerLineCommand("DMP", &commandDump);
      registerLineCommand("SET", &commandMastSet);
      registerLineCommand("OUT", &commandMapOutput);
      registerLineCommand("END", &commandEnd);
      registerLineCommand("MAP", &commandMapAspects);
      registerLineCommand("OVR", &commandOverride);

      break;
  }
}