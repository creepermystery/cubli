#define BRAKE       7
#define DIR        4
#define PWMR        3
#define START       8

void setup() {
  Serial.begin(9600);
  pinMode(BRAKE, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(DIR, OUTPUT);

  
  analogWrite(PWMR, 20);
  delay(2000);
}

void loop() {
}
