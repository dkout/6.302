#include <math.h>

//Lab00 Variables To Modify ******************************
float KpA =0.1; // Proportional Gain for Angle Error
float KpV = 0.0;  // Proportional Gain for BackEMF Error
float nominalBackEMF = 0.5;
float direct = 2.5;  
float desiredDeltaBackEMF = 0.0; // Alternate + and - offset from nominal
float desiredAngle = 0.1;
// motorCmd=direct+KpA*errorAngle+KpV*errorDeltaBackEMF

unsigned int deltaT = 10000;      // Sample period in microseconds.

// ***************************************************************
// Denfinition based on 6302 wiring.
#define alternatePeriod 2.0 // Time Between alts in seconds
#define angleSensorPin  A5
#define pwmVoltageSensorPin  A3
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
#define analogAverages 20

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.

// Variables for loop control
elapsedMicros loopTime; // loopTime used to force precise deltaT between starts
unsigned int headroom;  // Headroom=post execution time before next loop start.
boolean switchFlag;  // Used to monitor loop timing.
unsigned int alternateCount;  // Used to count loops since last alternate
float realTime = deltaT * 1.0e-6;

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

  // Set PWM frequency
  analogWriteFrequency(motorOutPWM,23437.5); // Changes several timers!!!
  
}

void loop() {  // Main code, runs repeatedly

  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);

  while (int(loopTime) < int(deltaT)) {}; // looptime is an elapsedtime var.
  loopTime = 0;

  // Monitor Output should be an exact square wave with frequency = 1/(2*deltaT)
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

  // Read analog values and average to reduce noise.
  int motorI = 0;
  int pwmI = 0;
  int angleI = 0;
  for (int i = 0; i < analogAverages; i++) {
    motorI += analogRead(motorVoltageSensorPin);
    pwmI += analogRead(pwmVoltageSensorPin);
    angleI += analogRead(angleSensorPin);
  }
  float motorV = scaleVadc * float(motorI) / float(analogAverages*adcMax);
  float pwmV = scaleVadc * float(pwmI) / float(analogAverages*adcMax);
  float angleV = scaleVadc * float(angleI) / float(analogAverages*adcMax);

  // backEMF = valtage across motor - voltage across motor internal resistance.
  // The voltage across the motor internal inductance assumed neglible.
  // We measure the voltage across an external current sense resistor, 
  //                      rsenseV = pwmV - motorV,
  // and a fraction of rsenseV is subtracted from motorV to get the backEMF.
  // That fraction is the ratio of the sense resistor to the internal
  // motor resistor (and must be calibrated by experiment).
  #define RmotorOverRsense 0.7
  float BackEMF = motorV - RmotorOverRsense * (pwmV - motorV);

  // Error is the difference between desired deltaBackEMF and 
  // the measured deltaBackEMF. 
  float measuredDeltaBackEMF = (BackEMF - nominalBackEMF);
  float errorV = (desiredDeltaBackEMF - measuredDeltaBackEMF);

  // Angle error
  float measuredAngle = -angleV + scaleVadc/1.9;
  float errorA = (desiredAngle - measuredAngle);

  // The Motor Command is a sum of a direct term and a gain*error
  float motorCmd = direct + KpA * errorA + KpV * errorV;

  // Limit the motor command and write it out.
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM, int((motorCmdLim / vDrive)*dacMax));
  //analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));

  // If the alternate flag is set, alternate every alternate period.
  alternateCount += 1;
  if (float(alternateCount)*realTime > alternatePeriod) {
    alternateCount = 0;
    desiredDeltaBackEMF = -desiredDeltaBackEMF;
    headroom = deltaT;
  };

  // Print out BackEMF in millivolts so that the serial plotter autoscales.
  Serial.println(1000.0*measuredAngle);
}
