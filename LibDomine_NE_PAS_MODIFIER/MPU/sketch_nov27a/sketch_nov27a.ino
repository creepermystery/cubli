#include <TimerOne.h>
int compteur=0;
int temps=0;


void setup() {
   Timer1.initialize(100000);
   Timer1.attachInterrupt(function);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
void function(){
  compteur=compteur+1;
  Serial.println(compteur)}
