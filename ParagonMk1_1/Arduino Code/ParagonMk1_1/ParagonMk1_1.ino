// Paragon code for Arduino Uno R3

#include "Configuration.h"
#include "FaultDetect.h"
#include "DataBridge.h"

//----------------------------------------------------------------
// User Settings
//----------------------------------------------------------------
int throttleValMin = 150; // Set higher than minimum rawThrottleVal
int throttleValMax =  800; // Set lower than maximum rawThrottleVal
int throttleValCount = throttleValMin;
float maxBusVoltage = 44.0; // Maximum Voltage for overvoltage protection
float minBusVoltage = 12.0;// Minimum voltage for undervoltage protection

//----------------------------------------------------------------
// Constants
//----------------------------------------------------------------
// Feedback Values
  int rawThrottleVal, directionReq, reset;
// Command Constants
  int vehicleDirection = 1; // 1 = forward
// PWM output to driver. 50% duty cycle = no movement
  int newPulseWidth; // As 16bit number, 0 = full reverse, 65535 = full forward, 32767 = 50% duty cycle
// Used for throttling serial actions
  unsigned long latchTime;
// Used for Fault Detection
  FaultDetect myFault(&fault1pin, &fault2pin);
// Used for sending data to custom processor
  DataBridge adb(9600);

//----------------------------------------------------------------
// Setup
//----------------------------------------------------------------
void setup() {
  // Set serial baud rate
   Serial.begin(9600);

  // Setup reset pins
  pinMode(resetDriverPower, OUTPUT);
  digitalWrite(resetDriverPower, LOW); // Active low reset that cuts power to A3941 IC
  pinMode(resetDriverPin, OUTPUT);
  digitalWrite(resetDriverPin, LOW); // Active low reset that puts A3941 to sleep
  
  
  // Lower level PWM register setups
  //----------------------------------------------------------------
  // Timer1 Setup: Phase and Frequency Correct PWM (Mode 8) for pin 9 (statusLED) and pin 10 (phasePin)
  // 20kHz switching frequency
  // See Tables 15-3, 15-4, and 15-5 in ATmega328P manual for more detail
  TCCR1A = _BV(COM1A1) | _BV(COM1B1); // Sets TOP = ICR1. PWM can be inverted by adding "_BV(COM1A0) | _BV(COM1B0)"
  TCCR1B = _BV(WGM13) | _BV(CS10); // Sets prescaler to 1
  ICR1 = 400; // Sets TOP value, i.e. maximum Timer1 value

  // Timer2 Setup: Phase Correct PWM (Mode 1) for pin 3 (pwmHighPin) and pin 11 (pwmLowPin)
  // 31.37kHz switching frequency 
  // See Tables 17-4, 17-7, 17-8, and 17-9 in ATmega328P manual for more detail
  TCCR2A =  _BV(WGM20) | _BV(COM2A1) | _BV(COM2B1); // Sets TOP = MAX = 0xFF. PWM can be inverted by adding "_BV(COM2A0) | _BV(COM2B0)"
  TCCR2B = _BV(CS20); // Sets prescaler to 1, PWM frequency will be 31.37kHz
  
  // Default bridge to powered down state - All gate drives off
  OCR1A = 0; // Sets 16-bit OC1A (statusLED) low
  OCR1BH = 0; // Sets 16-bit OC1B (phasePin) low
  OCR1BL = 0;
  OCR2A = 0; // Sets 8-bit OC2A (pwmLowPin) low
  OCR2B = 0; // Sets 8-bit OC2B (pwmHighPin) low
  digitalWrite(syncRecPin, LOW);

  //Setup PWM pins to output.
  pinMode(pwmHighPin, OUTPUT);
  pinMode(pwmLowPin, OUTPUT);
  pinMode(phasePin, OUTPUT);
  pinMode(syncRecPin, OUTPUT);

  // Setup input pins. Writing an input LOW turns off built in 20K pull up resistor
  pinMode(directionPin, INPUT);
  pinMode(resetInput, INPUT);
  pinMode(fault1pin, INPUT);
  pinMode(fault2pin, INPUT);

  digitalWrite(directionPin, LOW);
  digitalWrite(resetInput, LOW);
  digitalWrite(fault1pin, LOW);
  digitalWrite(fault2pin, LOW);

  // Set bridge control
  OCR2A = 255; // Sets 8-bit OC2A (pwmLowPin) high
  OCR2B = 255; // Sets 8-bit OC2B (pwmHighPin) high
  digitalWrite(syncRecPin, HIGH);
  
  // Turn on bridge outputs
  digitalWrite(resetDriverPower, HIGH);
  digitalWrite(resetDriverPin, HIGH);

}

//----------------------------------------------------------------
// Main Loop
//----------------------------------------------------------------
void loop() {
 
  int reset = digitalRead(resetInput);
  int busVoltageVal = analogRead(busVoltageSense);
  float busVoltage = busVoltageVal * (50.0 / 1023);
  
  // Turn off bridge outputs under these conditions
  if(reset == 0) { /*resetInput must be pulled high for proper operation*/
    digitalWrite(resetDriverPin, LOW);
    digitalWrite(resetDriverPower, LOW);
  } else {
    OCR2A = 255;
    OCR2B = 255;
    digitalWrite(syncRecPin, HIGH);
    digitalWrite(resetDriverPower, HIGH);
    digitalWrite(resetDriverPin, HIGH);
  }

 // Fault checking 
 //----------------------------------------------------------------
 if (myFault.hasFault()) {
      adb.announceFault(myFault.getLastFaultCode());
      if(myFault.getLastFaultCode() == 2) {
          OCR2A = 0;
          OCR2B = 0;
          digitalWrite(syncRecPin, LOW);
          OCR1B = 0;
          return;
      }
 } 
  
  // Throttle mapping
  //----------------------------------------------------------------
  rawThrottleVal = analogRead(throttleSense); // Reads current throttle value (0-1023)
  float throttleVoltage = rawThrottleVal * (5.0 / 1023); // Converts throttle value to voltage (0.0-5.0)

 // If throttleVal drops below current throttleValCount, output is decremented (once per loop) until throttleVal is >= throttleValCount. Can be eased more by adding more loops between decrements.
  if (rawThrottleVal >= throttleValCount) {
    throttleValCount = rawThrottleVal;
  } else {
    --throttleValCount;
  }
  
  int throttleVal = constrain(throttleValCount, throttleValMin, throttleValMax); // Ensures throttle stays in user set bounds (throttleValMin-throttleValMax)
  int throttleMapped = map(throttleVal, throttleValMin, throttleValMax, 0, 1023); // Maps throttle value according to user settings (0-1023)

  // Only change direction if throttle is low enough
  directionReq = digitalRead(directionPin);
  if(directionReq == !vehicleDirection) {
   if(throttleMapped < 10) {
     vehicleDirection = directionReq;
   }
  }

  // Map throttleCommand
  int throttleCommand;
  throttleCommand = map(throttleMapped, 0, 1023, 0, ICR1 / 2); // (0-200)
  
  // Remap throttleCommand according to vehicle direction, and add reverse speed limit
  if (!vehicleDirection) {
    throttleCommand = map(throttleCommand, 0, ICR1 / 2, ICR1 / 2, 0); 
    if (throttleCommand < (ICR1 / 4)) { // 50% reverse speed limit
      throttleCommand = (ICR1 / 4);
    }
  } else {
    throttleCommand = map(throttleCommand, 0, ICR1 / 2, ICR1 / 2, ICR1); 
  }

  OCR1B = throttleCommand; // FINALLY set the PWM register
  
  
  // Serial Communication 
  //-----------------------
  //Send data to the serial monitor
  if(millis()>latchTime) {
    adb.announceMotorControllerData(
    reset, 
    busVoltageVal, busVoltage,
    rawThrottleVal, throttleVal, throttleVoltage, throttleMapped, throttleCommand,
    vehicleDirection,
    OCR1BH, OCR1BL);
    adb.announceFault(myFault.getLastFaultCode());
    latchTime+=1000;
  }  
} 

