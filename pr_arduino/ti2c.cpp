#include "Arduino.h"
#include "ti2c.h"
#include <Wire.h>
#define SIZE 100

const char postfix = '$';

Ti2c::Ti2c(uint8_t i2caddr){
//initializer
  this->addr = i2caddr;
}

void Ti2c::sendStr(char buf[]){  
  Wire.beginTransmission(this->addr);	 // 
  Wire.write(buf);				// 1バイトをキューへ送信
  Wire.write(postfix);
  Wire.endTransmission();		// 送信完了  
}

void Ti2c::receiveStr(char buf[]){
  char b_buf[SIZE] = "";
  int i = 0;
  long j = 0;
   while(1){
     j++;
     if (j > 1000){
       sprintf(buf, "TimeOut");
       return;
     }
    Wire.requestFrom(this->addr, uint8_t(1));
        byte val;
        
    while (Wire.available()) {
      val = Wire.read();
      //Serial.print(val);
      if (val == postfix){
        b_buf[i++] = '\0';
        break;
      } else {
        b_buf[i++] = val;
      }
    }
    if (val == postfix){
      sprintf(buf, "%s", b_buf);
      break;
    }  
  } 
}
