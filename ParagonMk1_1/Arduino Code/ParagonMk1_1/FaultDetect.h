#ifndef FaultDetect_h
#define FaultDetect_h

#include "Arduino.h"

class FaultDetect {
  public:
    FaultDetect(const int*, const int*);
    boolean hasFault();
    int getLastFaultCode();
  private:
    int readFault(int, int);
    const int *_fault1pin;
    const int *_fault2pin;
    int _faultCode;
};
#endif
