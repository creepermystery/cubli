#define BRAKE       7
#define DIR        4
#define PWM        3
#define START       8

void setup() {
  Serial.begin(9600);
  pinMode(BRAKE, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(DIR, OUTPUT);
  delay(2000);
}

void loop() {
  digitalWrite(DIR, HIGH);
  analogWrite(PWM, 100);
  delay(3000);
  analogWrite(PWM, 20);
  delay(3000);
  digitalWrite(DIR, LOW);
  analogWrite(PWM, 20);
  delay(3000);
  analogWrite(PWM, 100);
  delay(3000);
}
