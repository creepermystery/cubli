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

  analogWrite(PWM, 100);
  delay(2000);
}

void loop() {
    delay(1000);
    digitalWrite(DIR, HIGH);

    delay(1000);
    digitalWrite(DIR, LOW);
}
