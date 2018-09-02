
#include <math.h>
#include "linmath.h"

// Set your direct term and gains here.
float angleOffsetV = -0.22;  // RESET THIS VALUE (-0.2->0.2) so precisely horizontal arm --> precisely zero angle
float directV1 = 2.9;  // RESET THIS VALUE with zero state gains so the prop is nominally horizontal
float directV2 = 2.5;

// Set Control Feedback gains 
float Kd1 = 5.3597; // Gain for arm angle state
float Kd2 = 1.9910e+01; // Gain for arm rotational velocity state
float Kd3 = 9.9097e+01;  // Gain for backEmf1 state
float Krd = 3.3788e+01; // Change Krd with K!!

// Set State Estimation gains.
float Ld1 = 1.0253e+02;  // Observer gain for angle Eqn
float Ld2 = 4.8075e+00;  // Observer gain for Omega Eqn
float Ld3 = 3.3350e-02;  // Observer gain for BackEMF Eqn

// Add Integral
float Kint = 0; //Gain for integral of error 

// BackEMF calibration.
float RmotorOverRsense = 0.8;  // RESET THIS value (0.5->1.5) to get bemf=zero when motor is stopped
float nominalBemfV = 1.14; // RESET THIS VALUE of motorBemf when horizontal, to use in  linearization


//FILL IN YOUR MATRIX VALUES BELOW for Observer Model:

// The discrete-time A matrix
mat3x3 Ad = {
    {  9.9551e-01,  -2.2354e-03,   6.7419e-04},
   {3.9910e-03,   1.0000e+00,   1.3494e-06},
   {1.9970e-06,   1.0000e-03,   1.0000e+00}
};
  
//The discrete time B matrix (column vector for SISO)
vec3 Bd = {
   3.9910e-03,
   7.9880e-06,
   2.6637e-09
};

//Output matrix C (row vector for SISO)
vec3 C = {
    0.0,   4.6032e-02,   2.9279e+00
};

// DONE WITH MATRICES FOR OBSERVER-BASED CONTROL

// State-Feedback Gains, set here or by assigning values above.
vec3 K = {
     Kd1, Kd2, Kd3
};

// Output error feedback gains for observer.
vec3 L = {
    Ld1, Ld2, Ld3
};

//User Might want to modify these variables ******************************
unsigned int deltaT = 1000;         // Sample period in microseconds.

#define pastSize 5
int past_size = 3;                 // interval for delta, larger=less noise, more delay.
unsigned long transferDt = 20000; // usecs between host updates  
float desiredAngleV;

float errorVintegral=0;                // Variable for integrating angle errors.
float integralMax = 200;            // Maximum value of the integral

// ***************************************************************
// ***************************************************************

//estimated_state vector
vec3 xhat =  {  3.3515e-03, 1.7548e-03, 3.3828e-01 };

// Control flags for estimation vs measured, reset est and integral
float est = 1;
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
#define motorOutPWM1 hbIn2A
#define motorOutPWM2 hbIn2B
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
bool first_time = false;
String config_message = "&A~Desired~5&C&S~Desired~A~-3.0~3.0~0.1&S~Set to 1 to Reset Estimate~Z~0~1.0~1.0&T~Angle~F4~-3.0~3.0&T~Error~F4~-6.0~6.0&T~MotorCmd~F4~0~5.0&T~X0~F4~-10~10&T~X1~F4~-10.0~10.0&T~X2~F4~-10~10&D~100&H~4&";
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

  // Set up side B for unidirectional, A for unidirectional.
  digitalWrite(hbIn1B, LOW);
  digitalWrite(hbIn2B, LOW);
  digitalWrite(hbIn1A, LOW);
  digitalWrite(hbIn2A, LOW);
  digitalWrite(hbMode, HIGH);

  // Integral Init.
  xhat[0] = 0;
  xhat[1] = 0;
  xhat[2] = 0; //zero out that stuff.
  errorVintegral = 0;

  // Set PWM frequency and nominal voltage
  pinMode(motorOutPWM1, OUTPUT);
  analogWriteFrequency(motorOutPWM1, 187500); // Teensy 3.0 pin 3 also changes to 23437.5 kHz
  analogWrite(motorOutPWM1, LOW);
  pinMode(motorOutPWM2, OUTPUT);
  analogWriteFrequency(motorOutPWM2, 187500); // Teensy 3.0 pin 3 also changes to 23437.5 kHz
  analogWrite(motorOutPWM2, LOW);
  
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);

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
  // Read average, scale, and offset sensor voltages (angle, motorVoltage, pwmVoltage)
  int angleI = 0;
  int motorI = 0;
  int pwmI = 0;
  for (int i = 0; i < analogAverages; i++) {
    angleI += analogRead(angleSensorPin);
    motorI += analogRead(motorVoltageSensorPin);
    pwmI += analogRead(pwmVoltageSensorPin);
  }
  float angleV = -angleOffsetV-scaleVadc*float(angleI- analogAverages*adcCenter)/float(analogAverages*adcMax);
  float motorV = scaleVadc * float(motorI) / float(analogAverages*adcMax);
  float pwmV = scaleVadc * float(pwmI) / float(analogAverages*adcMax);

  // Compute deltaVemf.
  float backEmfV =  motorV - RmotorOverRsense * (pwmV - motorV);
  float deltaVemf = backEmfV - nominalBemfV;

  // Compute approximate integral of the error.
  float errorV = angleV - desiredAngleV;
  errorVintegral = errorVintegral + dTsec*errorV;
  
  // Estimate d(angle)/dt = omega approx=  (angleV[n] - angleV[n-m])/(m*DeltaT).
  float omegaV = (angleV - pastAngleV[past_size-1])*scaleDeriv;

  // Zero Estimate also zeros out integral
  if(zero_est){
    xhat[0] = 0;
    xhat[1] = 0;
    xhat[2] = 0;
    errorVintegral = 0;
  }

  // Update u[n] = Krd * r - K*xhat;
  float u;
  if (est==1){
    u = Krd*desiredAngleV - (K[0]*xhat[0] + K[1]*xhat[1] + K[2]*xhat[2] + Kint*errorVintegral);
  }else{
    u = Krd*desiredAngleV - (K[0]*angleV + K[1]*omegaV + K[2]*deltaVemf + Kint*errorVintegral);
  }
  
  // *************calculate the motor command signal here***********************
  float motorCmd1 = directV1 + u; //for use in actual board.
  float motorCmd2 = directV2 - u; //for use in actual board.
  float motorCmdLim1 = min(max(motorCmd1, 0), vDrive);
  analogWrite(motorOutPWM1,int((motorCmdLim1/vDrive)*dacMax));
  float motorCmdLim2 = min(max(motorCmd2, 0), vDrive);
  analogWrite(motorOutPWM2,int((motorCmdLim2/vDrive)*dacMax));
  
  // Update previous errors for next time.
  for (int i = past_size-1; i > 0; i--) pastAngleV[i] = pastAngleV[i-1];
  pastAngleV[0] = angleV;

  // Send data back to plotter
  if (loopCounter == numSkip) {  
    packStatus(buf, angleV, errorV, motorCmdLim1, xhat[0], xhat[1], xhat[2], float(headroom));
    Serial.write(buf,30);
    loopCounter = 0;
    headroom = deltaT;
  } else loopCounter += 1;

  // Update xhat for next time through the loop.
  // Compute difference between measured and estimate y
  float ydiff = angleV  - vec3_mul_inner(C,xhat);
  
  // xhat[n+1] = Ad*xhat[n] + Bd*u[n] + L*(y[n] - yhat[n]), yhat[n] = C*xhat[n]
  vec3 temp1, Axhat, Bu, Lydiff;

  mat3x3_mul_vec3(Axhat, Ad, xhat); // AxHat = Ad*xhat
  vec3_scale(Bu,Bd,u); // Bu = Bd*u
  vec3_scale(Lydiff, L, ydiff); // Lydiff = L*(y-yhat) 
  vec3_add(temp1,Axhat,Bu); // temp1 = Ad*xhat+B*u
  vec3_add(xhat, temp1, Lydiff); // xhat[n+1] = Ad*xhat+Bd*u + L*(y-yhat) 
}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;
  // Zero past errors
  for (int i = past_size-1; i >= 0; i--) pastAngleV[i] = 0;
  errorVintegral = 0.0;
  
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
//      Krd = val;
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
  //memcpy(&buf[n],&h,sizeof(h));
  //n+=sizeof(h);
  //memcpy(&buf[n],&i,sizeof(i));
  //n+=sizeof(i);
  
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

