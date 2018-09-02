#include <math.h>

// Variables to Modify ******************************
float Kp = 8;
float Kd = 6;
float direct = 3.2;
float sineAmpl = 0;
float firstFreq = 0.2;
float lastFreq = 5.0;

// Settings for reducing noise
#define analogAverages 5
#define pastSize 3     // interval for delta, larger=less noise, more delay.

// ***************************************************************
// Sample period and plotting period
unsigned int deltaT = 1000;      // Sample period in microseconds.
float deltaTSecs = deltaT * 1.0e-6;
float oneOverMdt = 1.0/(deltaTSecs*pastSize); // Divide deltas by interval.  

// Browser Monitor Configuration. *********************************
//String config_message  = "&A~Desired~5&C&S~K_P~P~0~20~0.05&S~K_D~D~0~10~0.01&S~K_I~I~0~200~0.1&S~SumMax~S~0~50~0.1&S~Direct~O~0~5~0.01&S~Desired~A~-1.5~1.5~0.01&T~ArmAngle~F4~-1.5~1.5&T~Error~F4~-5~5&T~Delta~F4~-5~5&T~Sum~F4~-50~50&T~MotorCmd~F4~0~5&H~4&";
String config_message = "&T~Freq~F4~0~10&T~Mag~F4~0~0.5&T~Phase~F4~-200~200&T~FreqCmd~F4~0~10&T~MagCmd~F4~0~1&T~PhaseCmd~F4~-200~200&D~100&H~4&";

// Definitation based on 6302 wiring.
#define angleSensorPin  A5
#define pwmVoltageSensorPin A3
#define motorVoltageSensorPin  A4
#define hbMode 8
#define hbIn1A 9
#define hbIn2A 10
#define hbIn1B 11
#define hbIn2B 12
#define motorOutPWM  hbIn2A
#define monitorPin 2  
#define dacRes 12   // Teensy 12-bit dac resolution, max val 4095
#define dacMax 4095
#define adcRes 14
#define adcCenter 8492 //Teensy ADC set to 14 bits, 16384/2 = 8192
#define adcMax 16383
#define analogAverages 5

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.

// Variables for loop control
elapsedMicros loopTime; // loopTime used to force precise deltaT between starts
unsigned int headroom;  // Headroom=post execution time before next loop start.
boolean switchFlag;  // Used to monitor loop timing.
uint32_t loopCounter;
uint32_t numSkip = 20;  // Number of loops to skip between host updates.
bool first_time = false;

// Variables for Conversion
#define deg2rad 0.0175    
#define twopi 6.2831853  
#define MAX_ANG 5000
#define MAX_FREQ_PTS 40

// Sweeper variables
float angles[MAX_ANG]; // storage for the responses
float motorCmds[MAX_ANG];
float freqs[MAX_FREQ_PTS];
float Fmultiplier;
float newFreq;
boolean newFreqFlg;
int freqIndex;
int periodInDts;
int numDts;
float sa, saOld, ca, caOld, sc, scOld, cc, ccOld; 
                             
// Storage for past values.
float pastError[pastSize+1];  // Should be larger array than pastSize.
char buf[60];  // 

// Initializes past values.
void setup() {
  Serial.begin(115200);
  
  // Set up inputs
  analogReadResolution(adcRes);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);

  // Set up output
  analogWriteResolution(dacRes);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  // Set up side B for external bidirectional, A for unidirectional.
  pinMode(hbIn1B,INPUT);
  digitalWrite(hbIn2B, HIGH);
  digitalWrite(hbIn1A, LOW);
  digitalWrite(hbMode, HIGH);
  analogWrite(motorOutPWM, LOW);
  
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);

  // Set PWM frequency
  analogWriteFrequency(motorOutPWM, 23437.5); // Teensy 3.0 pin 3 also changes to 23437.5 kHz 

}

void loop() {  // Main code, runs repeatedly
  
  // Reinitializes or updates from sliders on GUI.
  startup();
  
  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);

  while (int(loopTime) < int(deltaT)) {};
  loopTime = 0;
  
  // Monitor Output should be a square wave with frequency = 1/(2*deltaT) 
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

 if(newFreqFlg) { // Initialize for next frequency point, make sure equals even number of deltaTSecss.
    newFreqFlg = false;
    periodInDts = 2*int(0.5/(deltaTSecs*newFreq));
    if ((periodInDts < 10) || (freqIndex == MAX_FREQ_PTS)) {
      newFreq = firstFreq;
      freqIndex = 0; 
      periodInDts = 2*int(0.5/(deltaTSecs*newFreq));
    }
    newFreq = float(1.0)/(deltaTSecs*float(periodInDts)); // Make sure the period is even number dTsec's
    numDts = 0;
    sc = 0; cc = 0; scOld = 0; ccOld = 0;
    sa = 0; ca = 0; saOld = 0; caOld = 0;
  } else {
    numDts += 1;
  }

  float arg = (twopi*float(numDts))/float(periodInDts);
  float cosv = cos(arg);
  float sinv = sin(arg);
 
  /*************** Section of Likely User modifications.**********************/

  // Read analog values and average to reduce noise.
  int angleI = 0;
  for (int i = 0; i < analogAverages; i++) {
    angleI += analogRead(angleSensorPin);
  }
  float angleV = scaleVadc * float(angleI) / float(analogAverages*adcMax);

  // Compute error, and error deltas from past.
  float desired = sineAmpl *sinv;
  
  // Angle error
  float measuredAngle = -angleV + scaleVadc/1.9;
  float errorA = (desired - measuredAngle);
  float errorAdelta = (errorA-pastError[pastSize-1])*oneOverMdt;

  // The Motor Command is a sum of a direct term and a gain*error
  float motorCmd = direct + Kp * errorA + Kd * errorAdelta;

  // Limit and write out motor command.
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM,int((motorCmdLim/vDrive)*dacMax));
  
  // Update previous angles for next time.
  for (int i = pastSize-1; i > 0; i--) pastError[i] = pastError[i-1];
  pastError[0] = errorA;
  
  /******** End Section of Likely User modifications.**********************/
   
  // Update the accumulating sine and cosine coeffs.
  float magA=0, magC=0;
  int numDtsMod = numDts % MAX_ANG;
  angles[numDtsMod] = angleV;
  motorCmds[numDtsMod] = motorCmd;
  sc += motorCmd*sinv;
  cc += motorCmd*cosv;
  sa += angleV*sinv;
  ca += angleV*cosv;
  int backAperiod = numDts - periodInDts; 
  if(backAperiod >= 0) { // Remove the first point if we've accumulated more than a period.
    backAperiod = backAperiod % MAX_ANG;
    sc -= motorCmds[backAperiod]*sinv; cc -=  motorCmds[backAperiod]*cosv;
    sa -= angles[backAperiod]*sinv;  ca -=  angles[backAperiod]*cosv;
    if (numDts % periodInDts == 0) { // Check convergence at periods (sin = 0).
        magC = sqrt(sc*sc + cc*cc);
        magA = sqrt(sa*sa + ca*ca);
        // If two periods or more AND longer than 3 seconds, then next freq.
        if (((numDts / periodInDts) > 1)  &&  (numDts > 3000000/int(deltaT))) { 
          newFreqFlg = true;
        }
        else {
          scOld = sc; saOld = sa;
          ccOld = cc; caOld = ca;
        }
     }
  }

  if(newFreqFlg) {  //  Just finished a frequency, dump out results
    float phaseSys = atan2(ca,sa)/deg2rad;
    float phaseCmd = atan2(cc,sc)/deg2rad;
    float periodInDtsF = float(periodInDts);
    magA /= periodInDtsF;
    magC /= periodInDtsF;
    packStatus(buf, newFreq, magA, phaseSys, newFreq, magC, phaseCmd, float(headroom));
    Serial.write(buf,30);
    newFreq *= Fmultiplier;
    freqIndex += 1;
  } 
  
  if (loopCounter == numSkip) {  // Lines below are for debugging.
    loopCounter = 0;
    headroom = deltaT;
  } else loopCounter += 1;

}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;

  // Zero past errors
  for (int i = pastSize-1; i >= 0; i--) pastError[i] = 0;

  // Sweeper specific initializations.
  freqIndex = 0;
  newFreqFlg = true;
  newFreq = firstFreq;
  Fmultiplier = exp(log(lastFreq/firstFreq)/float(MAX_FREQ_PTS));
}


// Initializes the loop if this is the first time, or if reconnect sent
// from GUI.  Otherwise, just look for serial event.
void startup(){
  if (first_time) {
    while(Serial.available() > 0) Serial.read(); // Clear out rcvr.
    Serial.println(config_message);  // Send the configuration files.
    while (! Serial.available()) {}  // Wait for serial return.
    while(Serial.available() > 0) {   // Clear out rcvr.
        Serial.read();
        //char inChar = (char) Serial.read(); 
        //if (inChar == '\n') break;
    }
    init_loop();
    first_time = false;
  } else {
    serialEvent();
  }
}

// Simple serial event, only looks for disconnect character, resets loop if found.
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, ready to process.
    if (inChar == '~') { // Got a disconnect
      first_time = true;
      break;
    }
  }
}

// Load the serial output buffer.
void packStatus(char *buf, float a, float b, float c, float d, float e, float f, float g) {
  
  // Start Byte.
  buf[0] = byte(0);
  int n = 1; 
  
  memcpy(&buf[n],&a,sizeof(a));
  n+=sizeof(a);
  memcpy(&buf[n],&b,sizeof(b));
  n+=sizeof(b);
  memcpy(&buf[n],&c,sizeof(c));
  n+=sizeof(c);
  memcpy(&buf[n],&d,sizeof(d));
  n+=sizeof(d);
  memcpy(&buf[n],&e,sizeof(e));
  n+=sizeof(e);
  memcpy(&buf[n],&f,sizeof(f));
  n+=sizeof(f);
  memcpy(&buf[n],&g,sizeof(g));
  n+=sizeof(g);

  // Stop byte (255 otherwise unused).
  buf[n] = byte(255); 
}

