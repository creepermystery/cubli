#define BRAKE       2
#define DIR         4
#define PWM         3
#define START       8

int temps=0;
void Motor_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR, HIGH);
  }
  analogWrite(PWM, sp > 255 ? 255 : sp);
}

void setup() {
  Serial.begin(9600);
  //mise en place moniteur
  pinMode(BRAKE, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(START, HIGH);
  digitalWrite(BRAKE, HIGH);
}

void loop() {
  if(millis()-temps>1000 && millis()-temps<2000){
  Motor_control(200);}
  else if(millis()-temps>2000 && millis()-temps<4000){
  Motor_control(0);}
  else if(millis()-temps>4000 && millis()-temps<6000){
  Motor_control(-100);}
  else if(millis()-temps>6000 && millis()-temps<8000){
  Motor_control(0);}
  else if(millis()-temps>8000 && millis()-temps<10000){
  Motor_control(20);}
  //si on veut que ça recommence
  else if(millis()-temps>10000){
  temps=millis();}
  }
