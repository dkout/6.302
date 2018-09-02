#include <math.h>

// Variables To Modify ******************************
float direct = 2.8;
#define pastSize 1

// Variables to set without monitor
float Kp = 0.0; // Proprtional Gain for Angle Error
float Kd = 0.0; 
float Kbemf = 0.0;
float Ku = 0.0;
float Ki = 0.0;
float desired = 0.0;
// motorCmd=direct+KpA*errorAngle+KpV*errorDeltaBackEMF

unsigned int deltaT = 1000;      // Sample period in microseconds.
float realTime = deltaT * 1.0e-6;
float scaleD = 1.0/(realTime*pastSize); // Divide deltas by interval.  

// Pick Arduino or Browser Monitor **********************************
boolean useBrowser = true;
String config_message_30_bytes = "&A~DesiredAng~5&C&S~DesiredAng~A~-1~1~0.1&S~Kp~P~0~20~0.1&S~Kd~D~0~5~0.1&T~Angle~F4~-2.5~2.5&T~Error~F4~-5~5&T~Deriv~F4~-10~10&T~Cmd~F4~0~5&D~100&H~4&";
String config_message = config_message_30_bytes;

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
uint32_t loopCounter;
uint32_t numSkip = 20;  // Number of loops to skip between host updates.
bool first_time = false;

// Storage for past values.
float pastError[pastSize+1];  // Should be larger array than past_size.
char buf[60];  

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

  // Reinitializes or updates from sliders on GUI.
  startup();

  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);

  while (int(loopTime) < int(deltaT)) {}; // looptime is an elapsedtime var.
  loopTime = 0;

  // Monitor Output should be an exact square wave with frequency = 1/(2*deltaT)
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

  // Read analog values and average to reduce noise.
  int angleI = 0;
  for (int i = 0; i < analogAverages; i++) {
    angleI += analogRead(angleSensorPin);
  }
  float angleV = scaleVadc * float(angleI) / float(analogAverages*adcMax);

  // Angle error
  float measuredAngle = -angleV + scaleVadc/1.9;
  float errorA = (desired - measuredAngle);
  float errorAdelta = (errorA-pastError[pastSize-1])*scaleD;

  // The Motor Command is a sum of a direct term and a gain*error
  float motorCmd = direct + Kp * errorA + Kd*errorAdelta;

  // Limit the motor command and write it out.
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM, int((motorCmdLim / vDrive)*dacMax));
  //analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));
  
  // Update previous errors for next time.
  for (int i = pastSize; i > 0; i--) pastError[i] = pastError[i-1];
  pastError[0] = errorA;
  
  if (loopCounter == numSkip) {  
    if (useBrowser) {
      packStatus(buf, measuredAngle, errorA, errorAdelta, motorCmdLim, float(headroom));
      Serial.write(buf,22);
    } else {
      // Print out in millivolts so that the serial plotter autoscales.
      Serial.println(1000.0*measuredAngle);
    }
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
    inputString += inChar;
    // if the incoming character is a newline, ready to process.
    if (inChar == '\n') {
      processString(inputString);
      inputString = "";
      break;
    }
  }
}

void processString(String inputString) {
char St = inputString.charAt(0);
  inputString.remove(0,1);
  float val = inputString.toFloat();
  switch (St) {
    case 'P': 
      Kp = val;
      break;
    case 'D':
      Kd = val;
      break;  
    case 'E':
      Kbemf = val;
      break;
    case 'U':
      Ku = val;
      break;
    case 'I':
      Ki = val;
      break;  
    case 'O':  
      direct = val;
      break;
    case 'A':
      desired = val;
      break;
    case '~':
      first_time = true;
      break;
    default:
    break;  
  }
}

// Load the serial output buffer.
void packStatus(char *buf, float a, float b, float c, float d, float e) {
  
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
  /*
  memcpy(&buf[n],&f,sizeof(f));
  n+=sizeof(f);
  memcpy(&buf[n],&g,sizeof(g));
   n+=sizeof(g);
   */

  // Stop byte (255 otherwise unused).
  buf[n] = byte(255); 
}



