#include <math.h>

// Control Variables 
float Kp = 1.0;
float Kd = 0.0;
float directV = 0.0;
float desiredV = 0.0;
                
//Likely User Modified Variables ******************************
unsigned int deltaT = 1000;         // Sample period in microseconds.
unsigned int numSkip = 499;                   // Number of loops between transfers to monitor
     
          
// ***************************************************************
// Teensy plus wiring definitions.
#define angleSensorPin  A9
#define hbMode 8
#define hbIn1A 9
#define hbIn2A 10
#define hbIn1B 11
#define hbIn2B 12
#define motorOutPWM  hbIn1A
#define monitorPin 2  
#define dacRes 12   // Teensy 12-bit dac resolution, max val 4095
#define dacMax 4095
#define adcRes 14
#define adcCenter 8192 //Teensy ADC set to 14 bits, 16384/2 = 8192
#define adcMax 16383
#define angleAverages 4    

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.

// Variables for Conversion
#define deg2rad 0.0175    
#define twopi 6.2831853  

//Conversions.
float rad2deg = 1.0/deg2rad;        // 180/pi
float dTsec = 1.0e-6*deltaT;       // The period in seconds. 
float scaleD = 1.0/dTsec; // Divide deltas by interval.    
                             
// Paststate variables
float pastErrorV = 0.0;

// Variables for loop control
unsigned int loopCounter;
elapsedMicros loopTime; // Create elapsed time variable to ensure regular loops.
unsigned int headroom;  // Headroom is time left before loop takes too long.
boolean switchFlag;

// Set up pins
void setup() {
  Serial.begin(115200);
  
  // Set up inputs
  analogReadResolution(adcRes);
  pinMode(angleSensorPin, INPUT);

  // Set up output
  analogWriteResolution(dacRes);
  pinMode(motorOutPWM, OUTPUT);
  pinMode(hbMode, OUTPUT);
  digitalWrite(hbMode, HIGH);
  pinMode(hbIn2A, OUTPUT);
  digitalWrite(hbIn2A, HIGH);
  analogWriteFrequency(motorOutPWM, 23437.5); // Teensy 3.0 pin 3 also changes to 23437.5 kHz
  analogWrite(motorOutPWM, LOW);
  
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);

}

void loop() {  // Main code, runs repeatedly
  
  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);
  
  while (int(loopTime) < int(deltaT)) {};
  loopTime = 0;
  
  // Monitor Output should be a square wave with frequency = 1/(2*deltaT) 
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

 
  /*************** Section of Likely User modifications.**********************/
  // Read Angle, average to reduce noise.
  int angleI = 0;
  for(int i = 0; i < angleAverages; i++) angleI += analogRead(angleSensorPin);
  float angleV = scaleVadc*float(angleI- angleAverages*adcCenter)/float(angleAverages*adcMax);

  // Compute error, and error deltas from past.
  float errorV = (desiredV - angleV);
  float errorVdelta = (errorV-pastErrorV)*scaleD;

  // Compute and write out motor command.
  float motorCmd = directV + Kp*errorV + Kd*errorVdelta;     
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM,int((motorCmdLim/vDrive)*dacMax));
  analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));

  // Update past state
  pastErrorV = errorV;
  
  if (loopCounter == numSkip) { 
    //Serial.print(float(headroom)); 
    Serial.println(angleV);
    loopCounter = 0;
    headroom = deltaT;
  } else loopCounter += 1;

}



