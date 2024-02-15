#define BRAKE       7
#define DIR         4
#define PWM         3
#define START       8
void setup() {
  Serial.begin(9600);
  pinMode(BRAKE, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(DIR, OUTPUT);

  analogWrite(PWM, 20);
  delay(2000);

}

void loop() {
  digitalWrite(DIR, HIGH);
  delay(2000);
  digitalWrite(DIR, LOW);
  delay(2000);

}
