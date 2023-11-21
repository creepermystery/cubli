#define BRAKE       26

#define DIR1        4
#define PWM1        3

#define TIMER_BIT  8
#define BASE_FREQ  20000

void pwmSet(uint8_t concerned_pin, uint32_t value) {
  analogWrite(concerned_pin, value);
}

void Motor_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR1, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR1, HIGH);
  }
  pwmSet(PWM1, sp > 255 ? 255 : 255 - sp);
}

void setup() {
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  
  pinMode(DIR1, OUTPUT);
  Motor_control(0);
  
  delay(2000);
}

void loop() {
    Motor_control(10);
    delay(2000);
    Motor_control(30);
    delay(2000);
    Motor_control(0);
    delay(2000);
    Motor_control(-10);
    delay(2000);
    Motor_control(-30);
    delay(2000);
    Motor_control(0);
    delay(2000);
}
