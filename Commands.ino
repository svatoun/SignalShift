#include <NmraDcc.h>
#include "Common.h"

boolean handleSignals(ModuleCmd cmd);
ModuleChain signals("signals", 0, &handleSignals);

extern void commandReset();

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
  printMastOutputs(nMast - 1);
  printAspectMap(nMast - 1);
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
    printMastOutputs(m);
    printAspectMap(m);
  }
}

boolean isMastActive(int nMast) {
  int cnt = 0;
  for (int i = 0; i < maxOutputsPerMast; i++) {
    const oneLightOutputs& outConfig = *(lightConfiguration[i]);
    int o = outConfig[nMast];
    if ((o != ONA) && (o > -1)) {
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
    if ((o != ONA) && (o > -1)) {
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
  Serial.print(F("DEF:")); Serial.print(nMast + 1); Serial.print(':'); Serial.print(first + 1); Serial.print(':'); Serial.print(cnt);
  Serial.print(':'); Serial.println(signalMastNumberAddress[nMast]);
  Serial.print(F("MST:")); Serial.print(nMast + 1); Serial.print(':'); Serial.print(signalMastSignalSet[nMast]); Serial.print(':'); Serial.println(signalMastDefaultAspectIdx[nMast]);
}

void printMastOutputs(int nMast) {
  byte lights[maxOutputsPerMast];
  int cnt = 0;
  for (int i = 0; i < maxOutputsPerMast; i++) {
    const oneLightOutputs& outConfig = *(lightConfiguration[i]);
    lights[i] = outConfig[nMast];
    if (lights[i] < ONA) {
      cnt++;
    }
  }
  for (int x = 0; x < maxOutputsPerMast; x++) {
    int a = lights[x];
    if (a >= ONA) {
      continue;
    }
    int seq = x + 1;
    while ((seq < sizeof(lights)) && (lights[seq] == (1 + lights[seq - 1]))) {
      seq++;
    }
    int len = seq - x;
    if (len == cnt) {
      return;
    }
    if (len > 2) {
      Serial.print(F("OSQ:")); Serial.print(nMast + 1); Serial.print(':'); Serial.print(x); Serial.print(':'); Serial.print(a); Serial.print(':'); Serial.println(seq - x);
      x = seq - 1; // will increment to seq in loop reinit
    } else {
      Serial.print(F("OUT:")); Serial.print(nMast + 1); Serial.print(':'); Serial.print(x); Serial.print(':'); Serial.println(a);
    }
  }
}

int findSameAspect(int nMast) {
  byte aspectMap[maxAspects];

  int cv = START_CV_ASPECT_TABLE + (nMast * maxAspects);
  for (int x = 0; x < maxAspects; x++, cv++) {
    aspectMap[x] = Dcc.getCV(cv);
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
  int copyOf = findSameAspect(nMast);
  if (copyOf >= 0) {
    Serial.print(F("MCP:")); Serial.print(nMast + 1); Serial.print(':'); Serial.println(copyOf + 1);
  } else {
    byte aspectMap[maxAspects];
    int cnt = 0;
    int cv = START_CV_ASPECT_TABLE + (nMast * maxAspects);
    for (int x = 0; x < maxAspects; x++, cv++) {
      aspectMap[x] = Dcc.getCV(cv);
      if (aspectMap[x] < maxAspects) {
        cnt++;
      }
    }
    for (int x = 0; x < maxAspects; x++) {
      int a = aspectMap[x];
      int seq = x + 1;
      if (a >= maxAspects) {
        continue;
      }
      while ((seq < maxAspects) && (aspectMap[seq] == (1 + aspectMap[seq - 1]))) {
        seq++;
      }
      int len = seq - x;
      if (len == maxAspects) {
        break;
      }
      if (len > 2) {
        Serial.print(F("MSQ:")); Serial.print(nMast); Serial.print(':'); Serial.print(x); Serial.print(':'); Serial.print(a); Serial.print(':'); Serial.println(len);
        x = seq - 1; // will increment to seq in loop reinit
      } else {
        Serial.print(F("MAP:")); Serial.print(nMast); Serial.print(':'); Serial.print(x); Serial.print(':'); Serial.println(a);
      }
    }
  }
}

void commandDefineMast() {
  int nMast = nextNumber();
  if (nMast < 1 || nMast > NUM_SIGNAL_MAST) {
    Serial.println(F("Invalid mast ID"));
    return;
  }

  int firstOut = nextNumber();
  if (nMast < 1 || nMast > NUM_OUTPUTS) {
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

  int cvBase = START_CV_OUTPUT + nMast * SEGMENT_SIZE;

  for (int i = 0; i < maxOutputsPerMast; i++) {
    if (i >= numLights) {
      Dcc.setCV(i, ONA);
    } else {
      Dcc.setCV(i, firstOut + i - 1);
    }
  }

  // the signal set number
  Dcc.setCV(cvBase + 10, SIGNAL_SET_CSD_BASIC);
  // the default signal
  Dcc.setCV(cvBase + 11, 0);
  // number of addresses = bits
  Dcc.setCV(cvBase + 12, bits);

  Serial.print(F("Mast #")); Serial.print(nMast); Serial.print(F(" uses outputs ")); Serial.print(firstOut); Serial.print(F(" - ")); Serial.print(firstOut + numLights);
  Serial.print(F(" and ")); Serial.print(bits); Serial.println(F(" addresses."));
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

void commandMastSet() {
  int nMast = nextNumber();
  if (nMast < 1 || nMast > NUM_SIGNAL_MAST) {
    Serial.println(F("Invalid mast ID"));
    return;
  }
  int signalSet = nextNumber();
  if (signalSet < 0 || signalSet >= _signal_set_last) {
    Serial.println(F("Invalid signal set"));
    return;
  }
  int defaultAspect = nextNumber();
  int cv = START_CV_OUTPUT + (nMast - 1) * SEGMENT_SIZE;
  int bits = Dcc.getCV(cv + 12);
  Serial.print("Limit = "); Serial.println(1 << bits);
  if (defaultAspect < 0 || defaultAspect >= maxAspects || defaultAspect >= (1 << bits)) {
    Serial.println(F("Invalid default aspect"));
    return;
  }
  Dcc.setCV(cv + 10, signalSet);
  Dcc.setCV(cv + 11, defaultAspect);
  Serial.print(F("> MST:")); Serial.print(nMast); Serial.print(':'); Serial.print(signalMastSignalSet[nMast] = signalSet); Serial.print(':'); Serial.println(signalMastDefaultAspectIdx[nMast] = defaultAspect); 
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

boolean handleSignals(ModuleCmd cmd) {
  switch (cmd) {
    case initialize:
      registerLineCommand("SGN", &commandSetSignal);
      registerLineCommand("SET", &commandSetCV);
      registerLineCommand("GET", &commandGetCV);
      registerLineCommand("DEF", &commandDefineMast);
      registerLineCommand("INF", &commandPrintMastDef);
      registerLineCommand("DMP", &commandDump);
      registerLineCommand("MST", &commandMastSet);
      registerLineCommand("OUT", &commandMapOutput);

      break;
  }
}