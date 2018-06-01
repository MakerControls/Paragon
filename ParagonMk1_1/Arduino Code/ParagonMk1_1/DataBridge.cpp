#include "Arduino.h"
#include "DataBridge.h"

DataBridge::DataBridge(int baudRate) {
     //Be sure to declare serial in setup
}

void DataBridge::announceFault (int faultCode) {
    Serial.print("FaultFlag ");
    Serial.println(faultCode);
}

void DataBridge::announceMotorControllerData(
    int reset, 
    int busVoltageVal, float busVoltage,
    int rawThrottleVal, int throttleVal, float throttleVoltage, int throttleMapped, int throttleCommand,
    int vehicleDirection,
    int t1H, int t1L)
{
    
    Serial.println("\nData:");
    
    Serial.print("reset = ");
    Serial.println(reset); 
    
    Serial.print("busVoltageVal = ");
    Serial.print(busVoltageVal);
    Serial.print(", busVoltage = ");
    Serial.println(busVoltage);
    
    Serial.print("vehicleDirection = ");
    Serial.println(vehicleDirection);
    
    Serial.print("rawThrottleVal = ");
    Serial.print(rawThrottleVal);
    Serial.print(", throttleVal = ");
    Serial.print(throttleVal);
    Serial.print(", throttleMapped = ");
    Serial.print(throttleMapped);
    Serial.print(", throttleCommand = ");
    Serial.println(throttleCommand);
    
    Serial.print("throttleVoltage= ");
    Serial.print(throttleVoltage);
    
  
    Serial.print(", OCR1BH = ");
    Serial.print(t1H, HEX);
    Serial.print(", OCR1BL = ");
    Serial.println(t1L, HEX);
}
