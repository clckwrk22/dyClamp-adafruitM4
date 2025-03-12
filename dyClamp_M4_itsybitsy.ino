/*
dyClamp (dynamic clamp sketch for the pyClamp interface)
Copyright (C) 2019 Christian Rickert <rc.email@icloud.com>

Copyright (C) 2025 Paul Wagner <paul.j.wagner@fau.de>
Adaptation of the original dyClamp sketch for the OpenDynamicClamp 1.2.1 
hardware using the Adafruit ItsyBitsy M4 Express. 

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
MA 02110-1301, USA.

dyClamp was partially developed at Laboratory of Catherine Proenza,
Department of Physiology & Biophysics, University of Colorado,
Anschutz Medical Campus (NIH R01-HL088427).

The modifications to run with ODC1.2.1 hardware was made at the Institute for Physiology 1, FAU Erlangen-Nuernberg in the lab of Tobias Huth

This project is based on the original implementation for a dynamic clamp system
by Niraj S. Desai, Richard Gray, and Daniel Johnston. For details, please visit
their website: https://dynamicclamp.com.

Version 1.1
*/


typedef struct serialCmd {  //leave up here, doesn´t compile otherwise, will say the interpretString function is not properly declared as it references cmd
    int len = 0;
    int idx = 0;
    int sep = 0;
    float val = 0.0;
    String str = "";
} command;

command cmd; 

#include <math.h>
#include <elapsedMillis.h>      //Millis library for timekeeping on M4 Express
#include "Adafruit_TinyUSB.h"   //different usb protocol, only this works with pyClamp


// ----- ATSAMD51 Alternatives for Teensy Functions ----- //
#ifdef __SAMD51__

// Fast digital read: directly access the port register for a given pin.
// look up the Port mapping in variant.cpp (Appdata/Local)
// this is faster than the usual digitalRead cycle
static inline uint32_t digitalReadFast(uint32_t pin) {
  uint32_t port = g_APinDescription[pin].ulPort;        // e.g., 0 for PORTA, 1 for PORTB, etc.
  uint32_t bitMask = (1 << g_APinDescription[pin].ulPin);  // create mask for the pin
  return (PORT->Group[port].IN.reg & bitMask) ? HIGH : LOW; //checks if this specific bit is high or low
}

// Configure ADC hardware oversampling.
// This function disables the ADC, sets oversampling parameters via the AVGCTRL register,
// and then re-enables the ADC. Adjust the macros (ADC_AVGCTRL_SAMPLENUM, ADC_AVGCTRL_ADJRES)
// This is an approximation of the builtin teensy function
void configureADCOversampling(uint8_t samples) {
  // Disable ADC before reconfiguration.
  ADC0->CTRLA.bit.ENABLE = 0;
 //Wait for synchronisation
  while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL);

  // The oversampling sample count is (samples - 1). Also set the adjustment resolution based on samples, this is basically an averaging routine.
  uint8_t samplenum = samples - 1;
  uint8_t adjres;
  switch(samplenum) {
    case 1:  adjres = 0; break;
    case 2:  adjres = 1; break;
    case 4:  adjres = 2; break;
    case 8:  adjres = 3; break;
    case 16: adjres = 4; break;
    default: adjres = 0; break;
  }
  
  // Configure oversampling.
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM(samplenum) | ADC_AVGCTRL_ADJRES(adjres);
    while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL); //always write one of the sync lines
    
  ADC0->CTRLB.reg = (ADC0->CTRLB.reg & ~ADC_CTRLA_PRESCALER_Msk) | ADC_CTRLA_PRESCALER_DIV16; //overclocks the ADC to run faster, cutting down cycle time - only produces faster results to DIV16
    while (ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
  //Enable ADC
  ADC0->SWTRIG.bit.START = 1;
  ADC0->CTRLA.bit.ENABLE = 1;

  //wait for ADC to be ready
  while(ADC0->SYNCBUSY.bit.ENABLE); 
}
#endif  // __SAMD51__

// ----- Existing Global Variables & Functions ----- //


bool report = false;            // report values live
int adcs = 0;                   // raw value from analog in [1]
int count = 0;                  // cycle counter for benchmarking
int dacs = 0;                   // raw value from analog out [1]
float pamps = 0.0;              // current output [pA]
float msecs = 0.0;              // time past since last cycle (dt) [ms]
float mvolts = 0.0;             // membrane potential [mV]

float calibras[] =  {50.0,      // Amplifier input gain (AMP_i) [mV/mV]
                     400.0,     // Amplifier output gain (AMP_o) [pA/V]
                     4.9756,       // ADC input slope (ADC_m) [mV/1]
                     -10087.453,  // ADC input intercept (ADC_n) [mV]
                     -624.3895,     // DAC output slope (DAC_m) [1/pA]
                     2108.4,    // DAC output intercept (DAC_n) [0-4095]
                     0.0};      // Voltage offset (VLT_d) [mV]
                     //values from GitHub of ODC1.2.1 board - calibration was set using a +/-3.3V sine wave and trimming the Pots, as described in the github page

float conducts[] =  {0.0,       // G_Shunt [nS]
                     0.0,       // G_H [nS]
                     0.0,       // G_Na [nS]
                     0.0,       // OU1_m [nS]
                     0.0,       // OU1_D [nS^2/ms]
                     0.0,       // OU2_m [nS]
                     0.0,       // OU2_D [nS^2/ms]
                     0.0};      // G_EPSC [nS]

float values[] =    {0.0,       // mvolts [mV]
                     0.0,       // pamps [pA]
                     0.0};      // msecs [µs]

elapsedMicros stepTime = 0;     // individual cycle time [µs]
elapsedMillis cmdTime = 0;      // time since last command check [ms]
elapsedMillis ttlTime = 0;      // time since last TTL trigger [ms]


 
//different pinout in M4 versus teensy 

const int adcPin = A3;           // analog in pin identifier
const int cmdIntvl = 200;        // command interval [ms]
const int dacPin = DAC0;         // analog out pin identifier
const int ttlPin = 7;           // trigger pin identifier
const int lenCals = sizeof(calibras) / sizeof(calibras[0]);
const int lenCons = sizeof(conducts) / sizeof(conducts[0]);
const int lenVals = sizeof(values) / sizeof(values[0]);

// (Other functions: clearBuffers, interpretString, readString, runCommand, 
// setConductance, setParameter, writeString, newArray, newString, etc. remain unchanged.)

void readString() {
  cmd.str = Serial.readStringUntil('\n') + '\n';
}

void runCommand(int exec) {
  static bool troper = false;
  troper = report;
  if (report)
    report = false;
  switch (exec) {
    case 0:
      break;
    case 1:
      writeString(newArray(2));
      break;
    case 2:
      report = !troper;
      break;
    default:
      break;
  }
  if (exec != 2)
    report = troper;
}

void setConductance(int idx, float val) {
  if (idx < lenCons)
    conducts[idx] = val;
}

void setParameter(int idx, float val) {
  if (idx < lenCals)
    calibras[idx] = val;
}

void writeString(String str) {
  Serial.print(str);
}

String newArray(int len) {
  String valstr = "";
  for (int i = lenCals - 1; i >= 0; i--) {
    int j = -1 * (i + 1);
    values[0] = j;
    values[1] = calibras[i];
    valstr += newString(values, len);
  }
  for (int k = 0; k < lenCons; k++) {
    int l = k + 1;
    values[0] = l;
    values[1] = conducts[k];
    valstr += newString(values, len);
  }
  return valstr;
}

String newString(float vals[], int len) {
  static const String cr = "\r";
  static const String tb = "\t";
  static const String lf = "\n";
  static String str;
  str = cr;
  for (int i = 0; i < len; i++) {
    if (i > 0)
      str += tb;
    str += String(vals[i]);
  }
  str += lf;
  return str;
}

void clearBuffers() {
  Serial.flush();
  while (Serial.available() > 0)
    Serial.read();
}


/* interprets a command string */
void interpretString(command cmd) {
    cmd.sep = cmd.str.indexOf("\t");
    if (cmd.str.startsWith("\r") && cmd.sep != -1) {  // check format
        cmd.len = cmd.str.length();
        cmd.idx = cmd.str.substring(1,cmd.sep).toInt();
        if (cmd.idx < 0) {          // negative index, update parameters
          cmd.idx = -1 * cmd.idx - 1;
          cmd.val = cmd.str.substring(cmd.sep + 1, cmd.len).toFloat();
          setParameter(cmd.idx, cmd.val);
        } else if (cmd.idx > 0) {   // positive index, update conductances
          cmd.idx = cmd.idx - 1;
          cmd.val = cmd.str.substring(cmd.sep + 1, cmd.len).toFloat();
          setConductance(cmd.idx, cmd.val);
        } else {                    // zero index, execute command instead
          cmd.val = cmd.str.substring(cmd.sep + 1, cmd.len).toInt();
          runCommand(cmd.val);
        }
    }
}



// Forward declarations for functions used in loop
float Shunting(float mvolts);
float HCN(float mvolts, float conduct);
float Sodium(float mvolts);
float OrnsteinUhlenbeck(float mvolts);
float EPSC(float mvolts);
void UpdateEpscTrain();
void GenerateGaussianNumbers();
void GenerateSodiumLUT();
void GenerateHcnLUT();


// ----- setup() and loop() ----- //

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  clearBuffers();
  cmd.str = "";  // Initialize the command string variable

  analogReadResolution(12);

#ifdef __SAMD51__
  // On the ATSAMD51, configure the ADC for hardware oversampling (e.g., 4 samples).
  configureADCOversampling(1);
#else
  // On a Teensy 3.6, use the native function.
  analogReadAveraging(4);
#endif

  analogWriteResolution(12);
  pinMode(ttlPin, INPUT);

  GenerateGaussianNumbers();
  GenerateSodiumLUT();
  GenerateHcnLUT();
}

void loop() {
  // Read membrane potential
  adcs = analogRead(adcPin);
  mvolts = calibras[2] / calibras[0] * adcs + calibras[3] / calibras[0] + calibras[6];

  // Calculate cycle interval
  msecs = 0.001 * float(stepTime);
  stepTime = 0;

  // Reset current
  pamps = 0.0;

  // Add currents (Shunting, HCN, Sodium, Ornstein–Uhlenbeck, EPSC)
  if (conducts[0] > 0) {
    pamps += Shunting(mvolts);
  }
  pamps += HCN(mvolts, conducts[1]);
  if (conducts[2] > 0) {
    pamps += Sodium(mvolts);
  }
  if (conducts[3] > 0 || conducts[5] > 0) {
    pamps += OrnsteinUhlenbeck(mvolts);
  }
  if (conducts[7] > 0) {
#ifdef __SAMD51__
    if (((ttlTime > 2) == HIGH) && digitalReadFast(ttlPin)) {
#else
    if ((digitalRead(ttlPin) == HIGH) && (ttlTime > 2)) {
#endif
      UpdateEpscTrain();
      ttlTime = 0;
    }
    pamps += EPSC(mvolts);
  }

  // Write calculated current to DAC output.
  dacs = int(calibras[4] / calibras[1] * pamps + calibras[5]);
  dacs = constrain(dacs, 0, 4095);
  analogWrite(dacPin, dacs);

  // Handle serial commands and live reporting.
  if (cmdTime > cmdIntvl) {
    if (Serial.available() > 4) {
      readString();
      writeString(cmd.str);
      interpretString(cmd);
    }
    if (report && Serial.availableForWrite() > 63) {
      values[0] = mvolts;
      values[1] = pamps;
      values[2] = 1000.0 * msecs;
      writeString(newString(values, lenVals));
    }
    cmdTime = 0;
  }
}
