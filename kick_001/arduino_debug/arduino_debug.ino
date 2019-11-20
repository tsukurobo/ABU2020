//23000 pulse / spin (maxon)
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"

// main for testing.
uint8_t addr = 0x11;
Ti2c ti2c = Ti2c(addr);
Ti2c ti2c2 = Ti2c(0x12);

char buf[100];
int pw = -30;
long enc = 0;
void setup(){
  Wire.begin();
  Serial.begin(115200);
  Serial.println("start");
}

void loop(){
  //Wire.requestFrom(addr, 1);
//  pw++;
//  if (pw > 50)
//    pw = -50;
  if (enc > 115000UL){//(23000*5)){
    //ti2c.sendStr("0");
    //delay(1000);
    pw = -50;
  }
  if (enc < 1) {
    //ti2c.sendStr("0");
    //delay(1000);
    pw = 50;
  }
  sprintf(buf, "%d", pw);
  Serial.println(buf);
  ti2c.sendStr(buf);
  ti2c.receiveStr(buf);
  ti2c2.sendStr(buf);
  ti2c2.receiveStr(buf);
  //Serial.println(buf);
  enc = atol(buf);
  Serial.println(enc);
  delay(100);
//  delay(1000);
//  ti2c.sendStr("1234"); 
//  delay(1000);
//  ti2c.sendStr("2"); 
//  delay(1000);

}
