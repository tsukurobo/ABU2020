#ifndef IseMotorDriver_h
#define IseMotorDriver_h
#include "ti2c.h"
#include "Arduino.h"

class IseMotorDriver {
  public:
    IseMotorDriver(uint8_t i2caddr);
    void setSpeed(int power);
    long encoder();
  private:
    uint8_t addr;
    Ti2c ti2c;
};
#endif
