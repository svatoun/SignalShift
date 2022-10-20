typedef LightFunction AspectDefinition[maxOutputsPerMast];
typedef byte AspectDefinitionBytes[maxOutputsPerMast];
typedef AspectDefinitionBytes SignalSet32[32];

#define STRIP_OFF LOFF, LOFF, LOFF, LOFF, LOFF
#define STRIP_40 STRIP_OFF
#define STRIP_60 LOFF, LOFF, LON, LOFF, LOFF
#define STRIP_80 LOFF, LON, LOFF, LOFF, LOFF
#define B54 B(blinking54)
#define B108 B(blinking108)
#define B22 B(blinking22)
#define L___ LOFF

const SignalSet32 csdBasicAspects PROGMEM = {
  { L___, L___, LON, L___, L___, STRIP_OFF },   // Aspect 0: Stuj
  { L___, LON, L___, L___, L___, STRIP_OFF },   // Aspect 1: Volno
  { LON, L___, L___, L___, L___, STRIP_OFF },   // Aspect 2: Výstraha,
  { B54, L___, L___, L___, L___, STRIP_OFF },   // Aspect 3: Očekávej 40
  { B108, L___, L___, L___, L___, STRIP_OFF },  // Aspect 4: Očekávej 60
  { L___, B54, L___, L___, L___, STRIP_OFF },   // Aspect 5: Očekávej 80
  { L___, L___, L___, L___, LON, STRIP_OFF },   // Aspect 6: 40 a volno
  { LON, L___, L___, L___, LON, STRIP_OFF },    // Aspect 7: 40 a výstraha
  { B54, L___, L___, L___, LON, STRIP_OFF },    // Aspect 8: 40 a očekávej 40
  { B108, L___, L___, L___, LON, STRIP_OFF },   // Aspect 9: 40 a očekávej 60
  { L___, B54, L___, L___, LON, STRIP_OFF },    // Aspect 10: 40 a očekávej 80
  { L___, LON, L___, L___, LON, STRIP_60 },     // Aspect 11: 60 a volno
  { LON, L___, L___, L___, LON, STRIP_60 },     // Aspect 12: 60 a výstraha
  { B54, L___, L___, L___, LON, STRIP_60 },     // Aspect 13: 60 a očekávej 40
  { B108, L___, L___, L___, LON, STRIP_60 },    // Aspect 14: 60 a očekávej 60
  { L___, B54, L___, L___, LON, STRIP_60 },     // Aspect 15: 60 a očekávej 80
  { L___, LON, L___, L___, LON, STRIP_80 },     // Aspect 16: 80 a volno
  { LON, L___, L___, L___, LON, STRIP_80 },     // Aspect 17: 80 a výstraha
  { B54, L___, L___, L___, LON, STRIP_80 },     // Aspect 18: 80 a očekávej 40
  { B108, L___, L___, L___, LON, STRIP_80 },    // Aspect 19: 80 a očekávej 60
  { L___, B54, L___, L___, LON, STRIP_80 },     // Aspect 20: 80 a očekávej 80
  { L___, LON, L___, LON, L___, STRIP_OFF },    // Aspect 21: Opakovaná volno
  { LON, L___, L___, LON, L___, STRIP_OFF },    // Aspect 22: Opakovaná výstraha
  { B54, L___, L___, LON, L___, STRIP_OFF },    // Aspect 23: Opakovaná očekávej 40
  { B108, L___, L___, LON, L___, STRIP_OFF },   // Aspect 24: Opakovaná očekávej 60
  { L___, B54, L___, LON, L___, STRIP_OFF },    // Aspect 25: Opakovaná očekávej 80
  { LON, LON, LON, LON, LON, LON, LON, LON, LON, LON },  // Aspect 26: ----------------------------
  { STRIP_OFF, LON, L___, L___, L___, L___ },   // Aspect 27: Posun zakázán
  { L___, L___, L___, LON, L___, STRIP_OFF },   // Aspect 28: Posun dovolen
  { L___, L___, LON, LON, L___, STRIP_OFF },    // Aspect 29: Posun dovolen - nezabezpečený
  { L___, L___, L___, B54, L___, STRIP_OFF },   // Aspect 30: Opatrně na přivolávací návěst bez červené
  { L___, L___, LON, B54, L___, STRIP_OFF },    // Aspect 31: Opatrně na přivolávací návěst
};

const SignalSet32 csdIntermediateAspects PROGMEM = {
  { L___, L___, LON, L___, L___, STRIP_OFF },  // Aspect 0: Stuj
  { L___, LON, L___, L___, L___, STRIP_OFF },  // Aspect 1: Volno
  { LON, L___, L___, L___, L___, STRIP_OFF },  // Aspect 2: Výstraha,
  { B54, L___, L___, L___, L___, STRIP_OFF },  // Aspect 3: Očekávej 40
  { L___, LON, L___, LON, LON, STRIP_OFF },    // Aspect 4: 40 a opakovaná volno             *
  { L___, LON, L___, LON, LON, STRIP_60 },     // Aspect 5: 60 a opakovaná volno             *
  { L___, L___, L___, L___, LON, STRIP_OFF },  // Aspect 6: 40 a volno
  { LON, L___, L___, L___, LON, STRIP_OFF },   // Aspect 7: 40 a výstraha
  { B54, L___, L___, L___, LON, STRIP_OFF },   // Aspect 8: 40 a očekávej 40
  { L___, LON, L___, LON, LON, STRIP_80 },     // Aspect 9: 80 a opakovaná volno             *
  { LON, L___, L___, LON, LON, STRIP_OFF },    // Aspect 10: 40 a opakovaná výstraha         *
  { L___, LON, L___, L___, LON, STRIP_60 },    // Aspect 11: 60 a volno
  { B54, L___, L___, LON, LON, STRIP_OFF },    // Aspect 12: 40 a opakovaná očekávej 40      *
  { B108, L___, L___, LON, LON, STRIP_OFF },   // Aspect 13: 40 a opakovaná očekávej 60      *
  { L___, B54, L___, L___, LON, STRIP_OFF },   // Aspect 14: 40 a opakovaná očekávej 80      *
  { LON, L___, L___, LON, LON, STRIP_60 },     // Aspect 15: 60 a opakovaná výstraha         *
  { L___, LON, L___, L___, LON, STRIP_80 },    // Aspect 16: 80 a volno
  { B54, L___, L___, L___, LON, STRIP_60 },    // Aspect 17: 60 a opakovaná očekávej 40      *
  { B108, L___, L___, L___, LON, STRIP_60 },   // Aspect 18: 60 a opakovaná očekávej 60      *
  { L___, B54, L___, L___, LON, STRIP_60 },    // Aspect 19: 60 a opakovaná očekávej 80      *
  { LON, L___, L___, LON, LON, STRIP_80 },     // Aspect 20: 80 a opakovaná výstraha         *
  { L___, LON, L___, LON, L___, STRIP_OFF },   // Aspect 21: Opakovaná volno
  { LON, L___, L___, LON, L___, STRIP_OFF },   // Aspect 22: Opakovaná výstraha
  { B54, L___, L___, LON, L___, STRIP_OFF },   // Aspect 23: Opakovaná očekávej 40
  { B108, L___, L___, LON, L___, STRIP_OFF },  // Aspect 24: Opakovaná očekávej 60
  { L___, B54, L___, LON, L___, STRIP_OFF },   // Aspect 25: Opakovaná očekávej 80
  { B54, L___, L___, LON, LON, STRIP_80 },     // Aspect 26: 80 a opakovaná očekávej 40      *
  { STRIP_OFF, STRIP_OFF },                    // Aspect 27: ----------------------------
  { L___, L___, L___, LON, L___, STRIP_OFF },  // Aspect 28: Posun dovolen
  { B108, L___, L___, LON, LON, STRIP_80 },    // Aspect 29: 80 a opakovaná očekávej 60
  { L___, L___, L___, B54, L___, STRIP_80 },   // Aspect 30: 80 a opakovaná očekávej 80
  { L___, L___, LON, B54, L___, STRIP_OFF },   // Opatrně na přivolávací návěst
};

const SignalSet32 csdEmbeddedAspects PROGMEM = {
  { L___, L___, LON, L___, L___, STRIP_OFF },                     // Aspect 0: Stuj
  { L___, LON, L___, L___, L___, STRIP_OFF },                     // Aspect 1: Volno
  { LON, L___, L___, L___, L___, STRIP_OFF },                     // Aspect 2: Výstraha,
  { B54, L___, L___, L___, L___, STRIP_OFF },                     // Aspect 3: Očekávej 40
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 4: ----------------------------           *
  { L___, L___, B22, L___, STRIP_OFF },                           // Aspect 5: Odjezdové návěstidlo dovoluje jízdu    *
  { L___, LON, L___, L___, L___, LON, L___, L___, L___, L___ },   // Aspect 6: Stůj s modrou                          *
  { LON, L___, L___, L___, LON, STRIP_OFF },                      // Aspect 7: 40 a výstraha
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 8:  ---------------------------           *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 9:  ---------------------------           *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 10: ---------------------------           *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 11: ---------------------------           *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 12: ---------------------------           *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 13: ---------------------------           *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 14: ---------------------------           *
  { STRIP_OFF, LON, L___, L___, L___, L___ },                     // Aspect 15: Sunout zakázáno opakovaná             *
  { L___, LON, L___, L___, L___, STRIP_OFF },                     // Aspect 16: Sunout zakázáno                       *
  { L___, L___, L___, LON, L___, L___, L___, L___, LON, L___ },   // Aspect 17: Sunout pomalu                         *
  { L___, L___, L___, LON, L___, STRIP_OFF },                     // Aspect 18: Sunout rychleji                       *
  { L___, LON, L___, L___, L___, L___, L___, L___, L___, LON },   // Aspect 19: Zpět                                  *
  { STRIP_OFF, LON, L___, L___, L___, LON },                      // Aspect 20: Zpět opakovaná                        *
  { L___, LON, L___, LON, L___, STRIP_OFF },                      // Aspect 21: Opakovaná volno
  { LON, L___, L___, LON, L___, STRIP_OFF },                      // Aspect 22: Opakovaná výstraha
  { B54, L___, L___, LON, L___, STRIP_OFF },                      // Aspect 23: Opakovaná očekávej 40
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 24: -----------------------               *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 25: Posun zakázán opakovaná               *
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 26: Na spádovišti se neposunuje           *
  { L___, L___, L___, L___, L___, LON, L___, L___, L___, L___ },  // Aspect 27: Posun zakázán
  { L___, L___, L___, LON, L___, STRIP_OFF },                     // Aspect 28: Posun dovolen
  { L___, L___, LON, LON, L___, STRIP_OFF },                      // Aspect 29: Posun dovolen - nezabezpečený
  { L___, L___, L___, B54, L___, STRIP_OFF },                     // Aspect 30: Opatrně na přivolávací návěst bez červené
  { L___, L___, LON, B54, L___, STRIP_OFF },                      // Aspect 31: Opatrně na přivolávací návěst
};

const SignalSet32 szdcBasicAspects PROGMEM = {
  { L___, L___, LON, L___, L___, STRIP_OFF },                     // Aspect 0: Stuj
  { L___, LON, L___, L___, L___, STRIP_OFF },                     // Aspect 1: Volno
  { LON, L___, L___, L___, L___, STRIP_OFF },                     // Aspect 2: Výstraha,
  { B54, L___, L___, L___, L___, STRIP_OFF },                     // Aspect 3: Očekávej 40
  { B108, L___, L___, L___, L___, STRIP_OFF },                    // Aspect 4: Očekávej 60
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 5: -----------------------                *
  { L___, L___, L___, L___, LON, STRIP_OFF },                     // Aspect 6: 40 a volno
  { LON, L___, L___, L___, LON, STRIP_OFF },                      // Aspect 7: 40 a výstraha
  { B54, L___, L___, L___, LON, STRIP_OFF },                      // Aspect 8: 40 a očekávej 40
  { B108, L___, L___, L___, LON, STRIP_OFF },                     // Aspect 9: 40 a očekávej 60
  { L___, B54, L___, L___, LON, STRIP_OFF },                      // Aspect 10: 40 a očekávej 80
  { L___, LON, L___, L___, LON, STRIP_60 },                       // Aspect 11: 60 a volno
  { LON, L___, L___, L___, LON, STRIP_60 },                       // Aspect 12: 60 a výstraha
  { B54, L___, L___, L___, LON, STRIP_60 },                       // Aspect 13: 60 a očekávej 40
  { B108, L___, L___, L___, LON, STRIP_60 },                      // Aspect 14: 60 a očekávej 60
  { L___, B54, L___, L___, LON, STRIP_60 },                       // Aspect 15: 60 a očekávej 80
  { LON, L___, L___, LON, LON, STRIP_OFF },                       // Aspect 16: 40 a opakovaná výstraha               *
  { B54, L___, L___, LON, LON, STRIP_OFF },                       // Aspect 17: 40 a opakovaná očekávej 40            *
  { B108, L___, L___, LON, LON, STRIP_OFF },                      // Aspect 18: 40 a opakovaná očekávej 60            *
  { LON, L___, L___, B54, L___, STRIP_OFF },                      // Aspect 19: Jízda podle rozhledových poměrů       *
  { LON, L___, L___, B54, LON, STRIP_OFF },                       // Aspect 20: 40 a jízda podle rozhledových poměrů  *
  { L___, LON, L___, LON, L___, STRIP_OFF },                      // Aspect 21: Opakovaná volno
  { LON, L___, L___, LON, L___, STRIP_OFF },                      // Aspect 22: Opakovaná výstraha
  { B54, L___, L___, LON, L___, STRIP_OFF },                      // Aspect 23: Opakovaná očekávej 40
  { B108, L___, L___, LON, L___, STRIP_OFF },                     // Aspect 24: Opakovaná očekávej 60
  { STRIP_OFF, STRIP_OFF },                                       // Aspect 25: -----------------------
  { STRIP_OFF, B108, L___, L___, L___, L___ },                    // Aspect 26: Jízda vlaku dovolena                  *
  { L___, L___, L___, L___, L___, LON, L___, L___, L___, L___ },  // Aspect 27: Posun zakázán
  { L___, L___, L___, LON, L___, STRIP_OFF },                     // Aspect 28: Posun dovolen
  { L___, L___, LON, LON, L___, STRIP_OFF },                      // Aspect 29: Posun dovolen - nezabezpečený
  { L___, L___, L___, B54, L___, STRIP_OFF },                     // Aspect 30: Opatrně na přivolávací návěst bez červené
  { L___, L___, LON, B54, L___, STRIP_OFF },                      // Aspect 31: Opatrně na přivolávací návěst
};

const SignalSet32 csdMechanicalAspects PROGMEM = {
  { L___, L___, LON, L___, L___, STRIP_OFF },  // Aspect 0: Stuj
  { STRIP_OFF, STRIP_OFF },                    // Aspect 1:  ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 2:  ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 3:  ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 4: ----------------------------
  { L___, L___, B22, L___, STRIP_OFF },        // Aspect 5: Odjezdové návěstidlo dovoluje jízdu
  { STRIP_OFF, STRIP_OFF },                    // Aspect 6: ----------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 7: ----------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 8: ----------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 9: ----------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 10: ----------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 11: ----------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 12: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 13: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 14: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 15: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 16: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 17: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 18: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 19: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 20: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 21: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 22: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 23: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 24: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 25: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 26: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 27: ---------------------------
  { L___, L___, L___, LON, L___, STRIP_OFF },  // Aspect 28: Posun dovolen
  { STRIP_OFF, STRIP_OFF },                    // Aspect 29: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 30: ---------------------------
  { STRIP_OFF, STRIP_OFF },                    // Aspect 31: ---------------------------
};

const MastTypeDefinition mastTypeDefinitions[] PROGMEM = {
  // Typ #1: 5-svetelne, vjezdove, zluta-zelena-cervena-bila-zluta
  {
    // Pocet kodu (max 32), pocet svetel (max 10), sada aspektu (viz SignalSet), vychozi kod (0.. pocet-kodu - 1).
    12, 5, SIGNAL_SET_CSD_BASIC, 0, 
    // Kontakty pro jednotlive lampy, cislovane od 1 do 10 (vcetne). 0 znamena ze se svetlo nepouzije.
    { 1, 2, 3, 4, 5 },
    // Jednotlive aspekty, cislovane od 1. Cislo kodu se prenasi budto 
    // - extended packetem prislusenstvi, nebo
    // - pomoci 2-koveho rozvoje na DCC adres navestidla, kde "primo" znaci 0 a "do odbocky" znaci 1; navestidlo obsadi vzdy tolik adres, aby 
    // - pomoci ovladani "primo", "do odbocky" vyhybek na DCC adresach navestidla, kde kody odpovidaji postupne:
    //   1. adresa "primo", 1. adresa "odbocka", 2. adresa "primo", 2. adresa "odbocka", 3. adresa "primo", ... az do vycerpani urceneho poctu kodu.
    {
      1,    // stuj
      2,    // volno
      7,    // 40 a volno (dolni zluta)
      29,   // posun
      // --------------------------------
      3,    // vystraha
      4,    // ocekavej 40
      8,    // 40 a vystraha
      9,    // 40 a ocekavej 40
      32,   // opatrne na privolavaci navest
      30,   // posun dovolen - bezabezpeceny
      0,    // test - zhasnuto
      255,  // test - vse rozsviceno
    }
  },
  // Definice #2: 4-svetelne, odjezdove; zelena-cervena-bila-zluta
  {
    8, 4, SIGNAL_SET_CSD_BASIC, 0, 
    { 0, 1, 2, 3, 4 },
    {
      1,    // stuj
      2,    // volno
      7,    // 40 a volno (dolni zluta)
      29,   // posun
      // --------------------------------
      32,   // opatrne na privolavaci navest
      30,   // posun dovolen - bezabezpeceny
      0,    // test - zhasnuto
      255,  // test - vse rozsviceno
    }
  },
  // Definice #3: 3-svetelne, odjezdove; cervena-zelena-bila
  {
    8, 3, SIGNAL_SET_CSD_BASIC, 0, 
    { 0, 1, 2, 3, 0 },
    {
      1,    // stuj
      2,    // volno
      2,    
      29,   // posun
      // --------------------------------
      32,   // opatrne na privolavaci navest
      30,   // posun dovolen - bezabezpeceny
      0,    // test - zhasnuto
      255,  // test - vse rozsviceno
    }
  },
  // Definice #4: 2-svetelne, seradovaci. Bila-modra.
  {
    2, 2, SIGNAL_SET_CSD_BASIC, 0, 
    { 0, 0, 0, 1, 0, 2 },
    {
      28,   // posun zakazan
      29,   // posun povolen
    }
  }
};
static_assert(((sizeof(mastTypeDefinitions) + sizeof(mastTypeDefinitions[0]) -1) / sizeof(mastTypeDefinitions[0]) < maxMastTypes), "Too many mast type definitions");

const byte mastTypeDefinitionCount = ((sizeof(mastTypeDefinitions) + sizeof(mastTypeDefinitions[0]) -1) / sizeof(mastTypeDefinitions[0]));


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

