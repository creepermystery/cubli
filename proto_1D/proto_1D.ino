

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

float err = 0;
float deriv = 0;
float integ = 0;

int power = 0;

unsigned long currentTime = millis();
unsigned long previousTime = millis();
unsigned long deltaT = 0;

float KP = 15;
float KI = 0;
float KD = 75;

int possibleHallucination = 0;

void loop()
{
    if (MPU_N.read())
  { 
    timer = millis();

   

    // Calculate Pitch, Roll and Yaw

    roll = MPU_N.m_fusedEulerPose[VEC3_X] * RAD_TO_DEGREE;
    pitch = MPU_N.m_fusedEulerPose[VEC3_Y] * RAD_TO_DEGREE;
    yaw = MPU_N.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE;// On récupère le lacet
  
    currentTime = millis();                 // On remet les variables temporelles en place
    deltaT = currentTime - previousTime;
    previousTime = currentTime;

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
