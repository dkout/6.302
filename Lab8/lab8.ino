#include <math.h>
// Calibration Variables
float directV = 2.9;  // RESET THIS VALUE with zero state gains so the prop is nominally horizontal
float nominalBemfV = 0.65; // RESET THIS VALUE of motorBemf when horizontal, to use in  linearization
float angleOffsetV = 0.05;  // RESET THIS VALUE precisely horizontal arm --> precisely zero angle

// Gains
float Kangle = 8.0; // Gain for arm angle state
float Komega = 3.5; // Gain for arm rotational velocity state
float Kemf= 1.6;  // Gain for backEmf state
float Kr = 8.0; // Change Kr with K!! 

// Loop timing, Derivative and integral variables
unsigned int deltaT = 1000;         // Sample period in microseconds.

#define pastSize 5                 // interval for delta, larger=less noise, more delay.
float dTsec = 1.0e-6*deltaT;       // The period in seconds. 
float scaleDeriv = 1.0/(dTsec*pastSize); // Divide deltas by interval.   

float errorVintegral;                // Variable for integrating angle errors.
float integralMax = 200;            // Maximum value of the integral

// ***************************************************************
// Variables for loop control 
elapsedMicros loopTime; // loopTime used to force precise deltaT between starts
unsigned int headroom;  // Headroom=post execution time before next loop start.
boolean switchFlag;  // Used to monitor loop timing.
unsigned int alternateCount;  // Used to count loops since last alternate
int loopCounter;
#define numSkip 20
bool first_time = false;
String config_message = "&A~Desired~5&C&S~Desired~A~-3.0~3.0~0.1&T~Theta~F4~-3.0~3.0&T~Omega~F4~-10.0~10.0&T~Vbemf~F4~-5.0~5.0&T~Error~F4~-3.0~3.0&T~MCmd~F4~0~5&D~100&H~4&";
                        
// Storage for past values.
float pastAngleV[pastSize]; 

// Storage for browser communication
char buf[60]; 
float desiredAngleV = 0;

// Definition based on 6302 wiring.
#define alternatePeriod 1.0 // Time Between alts in seconds
#define pwmVoltageSensorPin  A3
#define motorVoltageSensorPin  A4
#define angleSensorPin  A8
#define hbMode 4
#define hbIn1A 5
#define hbIn2A 6
#define hbIn1B 9
#define hbIn2B 10
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

// Set up pins
void setup() {
  Serial.begin(115200);

  // Loop period Monitor
  pinMode(monitorPin, OUTPUT);

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

  // Set up outputs
  analogWriteResolution(dacRes);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(A14, OUTPUT);
 
  // Set up side B for external bidirectional, A for unidirectional.
  digitalWrite(hbIn1B,HIGH);
  digitalWrite(hbIn2B, HIGH);
  digitalWrite(hbIn1A, LOW);
  digitalWrite(hbIn2A, LOW);
  digitalWrite(hbMode, HIGH);

  // Set PWM frequency
  analogWriteFrequency(motorOutPWM,187500); // Changes several timers!!!
  analogWrite(motorOutPWM, LOW);
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

  /*************** Section of Likely User modifications.**********************/
  // Read analog values and average to reduce noise.
  int angleI = 0;
  int motorI = 0;
  int pwmI = 0;
  for (int i = 0; i < analogAverages; i++) {
    angleI += analogRead(angleSensorPin);
    motorI += analogRead(motorVoltageSensorPin);
    pwmI += analogRead(pwmVoltageSensorPin);
  }
  // Note that the minus sign multiplying the angle is just for consistency with previous labs.
  float angleV = angleOffsetV-scaleVadc*float(angleI- analogAverages*adcCenter)/float(analogAverages*adcMax);
  float motorV = scaleVadc * float(motorI) / float(analogAverages*adcMax);
  float pwmV = scaleVadc * float(pwmI) / float(analogAverages*adcMax);
  
  // backEMF = valtage across motor - voltage across motor internal resistance.
  // The voltage across the motor internal inductance assumed neglible.
  // We measure the voltage across an external current sense resistor, 
  //                      rsenseV = pwmV - motorV,
  // and a fraction of rsenseV is subtracted from motorV to get the backEMF.
  // That fraction is the ratio of the sense resistor to the internal
  // motor resistor (and must be calibrated by experiment).
  // WE LINEARIZED THE BackEMF to THRUST relation, you must use nominalBemfV in the line below!!!!
  #define RmotorOverRsense 1.6
  float backEmfV =  motorV - RmotorOverRsense * (pwmV - motorV);
  float deltaVemf = backEmfV - nominalBemfV;
  
  // Compute omega = delta in arm angle divided by delta in time.
  float omegaV = (angleV - pastAngleV[pastSize-1])*scaleDeriv;

  // *************calculate the motor command signal here***********************
  // States are angleV (angle), omegaV (d(angleV)/dt), deltaVemf, Input is desiredAngleV
  float motorCmd = directV + Kr*desiredAngleV - (Kemf*deltaVemf + Komega*omegaV + Kangle*angleV);
                  
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM,int((motorCmdLim/vDrive)*dacMax));
  //analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));
  
  // Update previous errors for next time.
  for (int i = pastSize-1; i > 0; i--) pastAngleV[i] = pastAngleV[i-1];
  pastAngleV[0] = angleV;
  
  if (loopCounter == numSkip) {  
    float errorV = (desiredAngleV - angleV);
    packStatus(buf, angleV, omegaV, deltaVemf, errorV, motorCmdLim, float(headroom));
    Serial.write(buf,26);
    loopCounter = 0;
  } else loopCounter += 1;

}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;
  // Zero past errors
  for (int i = pastSize-1; i >= 0; i--) pastAngleV[i] = 0;
}


void processString(String inputString) {
char St = inputString.charAt(0);
  inputString.remove(0,1);
  float val = inputString.toFloat();
  switch (St) {
//    case 'B': 
//      K1 = val;
//      break;
//    case 'C':
//      K2 = val;
//      break;  
//    case 'D':
//      K3 = val;
//      break;
//    case 'U':
//      Ku = val;
//      break;
//    case 'O':  
//      directV = val;
//      break;
    case 'A':
      desiredAngleV = val;
      break;
    case '~':
      first_time = true;
      break;
    default:
    break;  
  }
}

// Load the serial output buffer.
void packStatus(char *buf, float a, float b, float c, float d, float e, float f) {
  
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
  //memcpy(&buf[n],&g,sizeof(g));
  //n+=sizeof(g);

  // Stop byte (255 otherwise unused).
  buf[n] = byte(255); 
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
  String inputString = ""; 
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, ready to process.
    if (inChar == '~') { // Got a disconnect
      first_time = true;
      break;
    }
    inputString += inChar;
    // if the incoming character is a newline, ready to process.
    if (inChar == '\n') {
      processString(inputString);
      inputString = "";
      break;
    }
  }
}

