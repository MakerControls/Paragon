#ifndef Configuration_h
#define Configuration_h

//Analog input pins
const int currentSense = 1;
const int speedLimitSense = 2;
const int throttleSense = 3;
const int brakeSense = 4;
const int busVoltageSense = 5;

//Digital pins
const int rx = 0;
const int tx = 1;
const int directionPin = 2; //input
const int pwmHighPin = 3; //output
const int resetDriverPin = 4; //output
const int fault2pin = 5; //input
const int fault1pin = 6; //input
const int resetInput = 7; //input from MISC connector
const int resetDriverPower = 8; //output
const int statusLED = 9; //output
const int phasePin = 10; //output
const int pwmLowPin = 11; //output
const int syncRecPin = 12; //output

#endif
