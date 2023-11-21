#define BRAKE       26

#define DIR1        4
#define PWM1        32
#define PWM1_CH     1

#define DIR2        15
#define PWM2        25
#define PWM2_CH     0

#define DIR3        5
#define PWM3        18
#define PWM3_CH     2

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
  Motor1_control(0);
  
  delay(2000);
}

void loop() {
    Motor1_control(10);
    delay(2000);
    Motor1_control(30);
    delay(2000);
    Motor1_control(0);
    delay(2000);
    Motor1_control(-10);
    delay(2000);
    Motor1_control(-30);
    delay(2000);
    Motor1_control(0);
    delay(2000);
}
