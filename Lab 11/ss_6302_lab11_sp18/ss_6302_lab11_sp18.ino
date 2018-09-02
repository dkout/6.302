
#include <math.h>
#include "linmath.h"

// Set your direct term and gains here.
float angleOffsetV = -0.22;  // RESET THIS VALUE (-0.2->0.2) so precisely horizontal arm --> precisely zero angle
float directV=2.9;  // RESET THIS VALUE with zero state gains so the prop is nominally horizontal
float RmotorOverRsense = 0.9;  // RESET THIS value (0.5->1.5) to get bemf=zero when motor is stopped
float nominalBemfV = 1.14; // RESET THIS VALUE of motorBemf when horizontal, to use in  linearization

float Kangle = 30.6795; // Gain for arm angle state
float Komega =4.1233; // Gain for arm rotational velocity state
float Kemf=11.8317;  // Gain for backEmf1 state
float Kint = 0;//0; //Gain for integral of error 
float Kr = 30.6795; // Change Kr with K!!

//--------------------------------
//--------------------------------
//FILL IN YOUR MATRIX VALUES BELOW for Observer:
// The L vector for converging the observer. These gains converge quite slowly
vec3 L = {
    0.3563, 40.5510, 13.6072
};

// A matrix
mat3x3 A = {
  {1.0, 1.000000000000000e-03, 5.439350010592381e-05},
  {0.0, 1.0, 0.108635363527522},
  {0.0, 0.0, 0.991659972612523}
};
  
//B matrix (vector)
vec3 B = {
      9.147506061303132e-08,
      2.742338963673659e-04,
      0.005020613501914
          };

//C matrix (vector)
vec3 C = {
    1,   0,   0
};


//Try to have some gain on K3 since it does help with overshoot
vec3 K = {
     Kangle,   Komega,   Kemf
};

//Likely User Modified Variables ******************************
unsigned int deltaT = 1000;         // Sample period in microseconds.

#define pastSize 5
int past_size = 3;                 // interval for delta, larger=less noise, more delay.
unsigned long transferDt = 20000; // usecs between host updates  
float desiredAngleV;


float errorVintegral=0;                // Variable for integrating angle errors.
float integralMax = 200;            // Maximum value of the integral

// ***************************************************************
// ***************************************************************

//--------------------------------
//--------------------------------
//FILL IN YOUR MATRIX VALUES BELOW for Observer:
// The L vector for converging the observer. These gains converge quite slowly

// ***************************************************************




float u = 0;
float y = 0; 

//estimated_state vector
vec3 xhat =  { 0, 0, 0 };


float est = 0;
float zero_est = 0;



// Teensy definitions
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
#define adcCenter 8192 //Teensy ADC set to 14 bits, 16384/2 = 8192, was 8492 for some reason...
#define adcMax 16383
#define analogAverages 5

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.

//Rest of Setup:
bool first_time;
String config_message = "&A~Desired~5&C&S~Desired~A~-3.0~3.0~0.1&S~Real=0/Est=1~R~0~1.0~1.0&S~Zero_Estimated:0~Z~0~1.0~1.0&T~Theta~F4~-3.0~3.0&T~Omega~F4~-10.0~10.0&T~Vbemf~F4~-5.0~5.0&T~EstTheta~F4~-3.0~3.0&T~EstOmega~F4~-10.0~10.0&T~EstVbemf~F4~-5.0~5.0&T~MotorCmd1~F4~0~5&T~Error~F4~0~5&D~100&H~4&";
float dTsec = 1.0e-6*deltaT;       // The period in seconds. 
float scaleDeriv = 1.0/(dTsec*past_size); // Divide deltas by interval.  
                         

int loopCounter;

// Storage for past values.
float pastAngleV[20];  // Should be larger array than past_size.
char buf[60];  // 

// Variables for loop control
uint32_t loop_counter;
#define numSkip 20  // Number of loops to skip between host updates.
elapsedMicros loopTime; // Create elapsed time variable to ensure regular loops.
unsigned int headroom;  // Headroom is time left before loop takes too long.
boolean switchFlag;

// Initializes past values.
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
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
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(A14, OUTPUT);

  // Set up side B for external bidirectional, A for unidirectional.
  digitalWrite(hbIn1B,HIGH);
  digitalWrite(hbIn2B, HIGH);
  digitalWrite(hbIn1A, LOW);
  digitalWrite(hbIn2A, LOW);
  digitalWrite(hbMode, HIGH);

  // Integral Init.
  errorVintegral = 0.0;

  // Set PWM frequency and nominal voltage
  analogWriteFrequency(motorOutPWM,187500); // Changes several timers!!!
  analogWrite(motorOutPWM,int(directV/vDrive)*dacMax);
  
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);

  // Number of loops between transfers to host.

  first_time = false;

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
  // Read Angle, average to reduce noise.
  int angleI = 0;
  int motorI = 0;
  int pwmI = 0;
  for (int i = 0; i < analogAverages; i++) {
    angleI += analogRead(angleSensorPin);
    motorI += analogRead(motorVoltageSensorPin);
    pwmI += analogRead(pwmVoltageSensorPin);
  }
  //for(int i = 0; i < angleAverages; i++) angleI += analogRead(angleSensorPin);
  //float angleV = scaleVadc*float(angleI- angleAverages*adcCenter)/float(angleAverages*adcMax);

  
  // Read motor voltage and pmw voltage
  float angleV = -angleOffsetV-scaleVadc*float(angleI- analogAverages*adcCenter)/float(analogAverages*adcMax);
  float motorV = scaleVadc * float(motorI) / float(analogAverages*adcMax);
  float pwmV = scaleVadc * float(pwmI) / float(analogAverages*adcMax);
  float backEmfV =  motorV - RmotorOverRsense * (pwmV - motorV);
  float deltaVemf = backEmfV - nominalBemfV;


  float errorV = angleV - desiredAngleV;
  errorVintegral = errorVintegral + dTsec*errorV;
  
  // Compute omega = delta in angle divided by delta in time.
  float omegaV = (angleV - pastAngleV[past_size-1])*scaleDeriv;

  float desiredAngleV_used = desiredAngleV;
  if(zero_est){
    desiredAngleV_used = 0;
  }
  if (est==1){
    u = Kr*desiredAngleV_used - (K[0]*xhat[0] + K[1]*xhat[1] + K[2]*xhat[2] +Kint*errorVintegral);
  }else{
    u = Kr*desiredAngleV_used - (K[0]*angleV + K[1]*omegaV +K[2]*(deltaVemf)+Kint*errorVintegral);
  }
    //input scale:
  y = angleV;
  
  // 1. UPDATE STATE
  vec3 temp1, temp2, temp3, temp4;
 
  // xhat[n+1] = A*xhat[n] + B*u[n] + L*(y[n] - C*xhat[n])::
  mat3x3_mul_vec3(temp1, A, xhat); // A*xhat
  vec3_scale(temp2,B,u); // B*u
  vec3_scale(temp3, L, (y-vec3_mul_inner(C,xhat))); // L*(y-yhat) = L*(y-C*xhat)
  //vec3_add(xhat,temp2,temp1); // A*xhat+B*u
  vec3_add(temp4,temp2,temp1); // A*xhat+B*u
  vec3_add(xhat, temp4,temp3); // xhatnew = A*xhat + B*u + L*(y-yhat) 
  if (zero_est){
    xhat[0] =0;
    xhat[1] = 0;
    xhat[2] = 0; //zero out that stuff.
  }
  // *************calculate the motor command signal here***********************
  float motorCmd = u + directV; //for use in actual board.
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM,int((motorCmdLim/vDrive)*dacMax));
  
  // Update previous errors for next time.
  for (int i = past_size-1; i > 0; i--) pastAngleV[i] = pastAngleV[i-1];
  pastAngleV[0] = angleV;
  
  if (loopCounter == numSkip) {  
    packStatus(buf, angleV, omegaV, deltaVemf,xhat[0], xhat[1], xhat[2],motorCmdLim,errorV, float(headroom));
    Serial.write(buf,38);
    loopCounter = 0;
    headroom = deltaT;
  } else loopCounter += 1;

}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;
  // Zero past errors
  for (int i = past_size-1; i >= 0; i--) pastAngleV[i] = 0;
}


void processString(String inputString) {
char St = inputString.charAt(0);
  inputString.remove(0,1);
  float val = inputString.toFloat();
  switch (St) {
    case 'R': 
      est = val;
      break;
    case 'Z':
      zero_est = val;
      break;  
//    case 'D':
//      K3 = val;
//      break;
//    case 'U':
//      Kr = val;
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
void packStatus(char *buf, float a, float b, float c, float d, float e, float f, float g, float h, float i) {
  
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
  memcpy(&buf[n],&h,sizeof(h));
  n+=sizeof(h);
  memcpy(&buf[n],&i,sizeof(i));
  n+=sizeof(i);
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

