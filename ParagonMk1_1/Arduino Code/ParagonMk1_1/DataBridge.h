#ifndef DataBridge_h
#define DataBridge_h

#include "Arduino.h"

class DataBridge {
  public:
    DataBridge(int);
    void announceFault(int);
    void announceMotorControllerData(
      int,
      int, float,
      int, int, float, int, int,
      int,
      int, int);
    };
#endif
