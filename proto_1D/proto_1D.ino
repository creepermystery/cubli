

#include <Wire.h>
#include "MPU9150Lib.h"

#define MPU_UPDATE_RATE  (10)

MPU9150Lib MPU_N; 
// Pins
const int PIN_PWM = 3;
const int PIN_DIR = 4;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float yaw, pitch, roll;
float accRoll;
float accRollInt;
float rollPlusUn;
bool newRoll = false;

int pos = 1500;

void setup() 
{
    Serial.begin(115200);
    Wire.begin();
    delay(1000);
    MPU_N.init(MPU_UPDATE_RATE); 
   
    // configure outputs
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
}

float previousroll = 0;

//<<<<<<< HEAD

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(BRAKE, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(START, OUTPUT);

    digitalWrite(BRAKE, HIGH);
    digitalWrite(START, HIGH);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

int yawValue = 0;
int prevYawValue = 0;

int err = 0;
int deriv = 0;
long integ = 0;
=======
float err = 0;
float deriv = 0;
float integ = 0;
>>>>>>> 556789a133bc519675f07d58b4ec6f1a33491d6d

int power = 0;

unsigned long currentTime = millis();
unsigned long previousTime = millis();
unsigned long deltaT = 0;

<<<<<<< HEAD
int KP = 15;
float KI = 0.00000;
int KD = 20;
=======
float KP = 15;
float KI = 0;
float KD = 75;
>>>>>>> 556789a133bc519675f07d58b4ec6f1a33491d6d

int possibleHallucination = 0;

void loop()
{
    if (MPU_N.read())
  { 
    timer = millis();

   

    // Calculate Pitch, Roll and Yaw

<<<<<<< HEAD
            err = yawValue;
            deriv = yawValue - prevYawValue;
            integ = integ + err * deltaT;
=======
    roll = MPU_N.m_fusedEulerPose[VEC3_X] * RAD_TO_DEGREE;
    pitch = MPU_N.m_fusedEulerPose[VEC3_Y] * RAD_TO_DEGREE;
    yaw = MPU_N.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE;// On récupère le lacet
  
    currentTime = millis();                 // On remet les variables temporelles en place
    deltaT = currentTime - previousTime;
    previousTime = currentTime;
>>>>>>> 556789a133bc519675f07d58b4ec6f1a33491d6d

    err = roll;                          // On prépare les variables du PID
    deriv = roll - previousroll;
    integ = integ + roll * deltaT;

    power = (KP*err + KI*integ + KD*deriv);  // On calcule le PWM

    previousroll = roll;

    Serial.print("\t pow:");
    Serial.print(power);
    Serial.print("\t roll:");
    Serial.print(roll);
    
    // Capper le PWM à 255
    if (power > 255) power = 255;
    else if (power < -255) power = -255;
    
    // Si le PWM est négatif, inverser la PIN_direction du moteur
    if (power >= 0) digitalWrite(PIN_DIR, HIGH);
    else
    {
        digitalWrite(PIN_DIR, LOW);
        power = -power;
    }

    Serial.print("\t pow:");
    Serial.print(power);

    // Arrêter le moteur si le cubli est tombé
    if ((roll > -35.0) && (roll < 35.0)) analogWrite(PIN_PWM, power);
    else analogWrite(PIN_PWM, 0);
            
    Serial.println("");

  
  }
}
