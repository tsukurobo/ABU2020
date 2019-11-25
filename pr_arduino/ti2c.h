#ifndef Ti2c_h
#define Ti2c_h
#include "Arduino.h"

class Ti2c{
  public:
    Ti2c();
    Ti2c(uint8_t i2caddr);
    void sendStr(char buf[]);
    void receiveStr(char buf[]);
    uint8_t addr;
};
#endif

