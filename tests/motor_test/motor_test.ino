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
<<<<<<< HEAD:motor_test/motor_test.ino
  digitalWrite(DIR, HIGH);
  delay(2000);
  digitalWrite(DIR, LOW);
  delay(2000);

=======
    delay(1000);
    digitalWrite(DIR, HIGH);

    delay(1000);
    digitalWrite(DIR, LOW);
>>>>>>> 556789a133bc519675f07d58b4ec6f1a33491d6d:tests/motor_test/motor_test.ino
}
