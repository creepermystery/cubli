#define BRAKE       2
#define DIR         4
#define PWM         3

void pwmSet(uint8_t concerned_pin, uint32_t value) {
  analogWrite(concerned_pin, value);
}

void Motor_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR, HIGH);
  }
  pwmSet(PWM, sp > 255 ? 255 : 255 - sp);
}

void setup() {
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  
  pinMode(DIR, OUTPUT);
  Motor_control(0);
  delay(2000);
}

void loop() {
    Motor_control(10);
    printf("2");
    delay(2000);
    Motor_control(30);
    printf("2");
    delay(2000);
    Motor_control(0);
    printf("2");
    delay(2000);
    Motor_control(-10);
    printf("2");
    delay(2000);
    Motor_control(-30);
    printf("2");
    delay(2000);
    Motor_control(0);
    printf("2");
    delay(2000);
}