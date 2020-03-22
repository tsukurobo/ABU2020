#include <ros.h>
#include <std_msgs/Int64MultiArray.h>

ros::NodeHandle  nh;
volatile  int temp, counter, counter2 = 0;

std_msgs::Int64MultiArray enc;
ros::Publisher chatter("enc", &enc);

ISR(PCINT1_vect,ISR_NOBLOCK){
  if((PINC & (1<<PINC0))^((PINC & (1<<PINC1))>>1))--counter2;
  else ++counter2;
  
  }

void setup() {
  //Serial.begin (9600);
  nh.initNode();
  PCICR |= (1<<PCIE1);
  PCMSK1 |=(1<<PCINT8);

  pinMode(A0, INPUT_PULLUP);
 
  pinMode(A1, INPUT_PULLUP);
 
  pinMode(2, INPUT_PULLUP);
 
  pinMode(3, INPUT_PULLUP);
  //interuptをセットアップする
  //
  attachInterrupt(0, ai0, RISING);
 
  //
  attachInterrupt(1, ai1, RISING);

  nh.advertise(chatter);
  enc.data_length = 2;
  enc.data = (long long int*)malloc(sizeof(long long int)*2);
}
 
void loop() {
  //Serial.println (counter);
  // カウンターの値を出力する
  if ( counter != temp ) {
    //Serial.println (counter);
    temp = counter;
  }

  enc.data[0] = counter;
  enc.data[1] = counter2;

  chatter.publish( &enc );
  nh.spinOnce();
}
 
void ai0() {
  if (digitalRead(3) == LOW) {
    counter++;
  } else {
    counter--;
  }
}
 
void ai1() {
  if (digitalRead(2) == LOW) {
    counter--;
  } else {
    counter++;
  }
}
