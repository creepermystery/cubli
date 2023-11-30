#define BRAKE       2
#define DIR         4
#define PWM         3
#define START       8

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
  pinMode(BRAKE, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(START, HIGH);
  digitalWrite(BRAKE, HIGH);
  delay(2000);
}

void loop() {
    Motor_control(200);
    delay(3000);
    Motor_control(0);
    delay(3000);
    Motor_control(-100);
    delay(3000);
    Motor_control(0);
    delay(3000);
    Motor_control(20);
    delay(3000);
}
