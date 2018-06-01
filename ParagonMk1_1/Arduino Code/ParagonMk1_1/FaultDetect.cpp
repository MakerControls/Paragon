#include "Arduino.h"
#include "FaultDetect.h"

FaultDetect::FaultDetect(const int* fault1pin, const int* fault2pin) {
   pinMode(*fault1pin, INPUT);
   pinMode(*fault2pin, INPUT);
   
  _fault1pin = fault1pin;
  _fault2pin = fault2pin;
}

int FaultDetect::readFault (int f1, int f2) {
  //Fault Flags thrown by A3941 driver chip:
  if (!(f1 || f2)) {
    return 0;  //0 = (FF1 = 0, FF2 = 0) : No Fault
  } else if (f1 && !f2) { 
    return 1;  //1 = (FF1 = 1, FF2 = 0) : Overtemperature (165 celcius) of A3941 driver chip. Fault is automatically cleared when temperature drops below recovery level.
  } else if (!f1 && f2) {
    return 2;  //2 = (FF1 = 0, FF2 = 1) : Short (Short to ground, short to supply, or shorted load). Fault latched until reset.
  } else if (f1 && f2) {
    return 3;  //3 = (FF1 = 1, FF2 = 1) : UnderVoltage (V5, VReg, or Bootstrap undervoltage). Fault latched until reset.
  }
}

boolean FaultDetect::hasFault() {
  int fault1val = digitalRead(*_fault1pin);
  int fault2val = digitalRead(*_fault2pin);
  _faultCode = FaultDetect::readFault(fault1val, fault2val);
  if (_faultCode == 0) {
    return false;
  }
  return true;
}

int FaultDetect::getLastFaultCode() {
  return _faultCode;
}
