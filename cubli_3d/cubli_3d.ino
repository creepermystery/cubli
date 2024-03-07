

#include <Wire.h>
#include "MPU9150Lib.h"

#define MPU_UPDATE_RATE  (10)

MPU9150Lib MPU_N;

// Pins
const int PIN_PWM_A = 3;
const int PIN_DIR_A = 4;
const int PIN_PWM_B = 5;
const int PIN_DIR_B = 6;
const int PIN_PWM_C = 9;
const int PIN_DIR_C = 10;

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
    pinMode(PIN_PWM_A, OUTPUT);
    pinMode(PIN_DIR_A, OUTPUT);
    pinMode(PIN_PWM_B, OUTPUT);
    pinMode(PIN_DIR_B, OUTPUT);
    pinMode(PIN_PWM_C, OUTPUT);
    pinMode(PIN_DIR_C, OUTPUT);
}

float errRoll = 0;
float derivRoll = 0;
float integRoll = 0;
float previousRoll = 0;

float errPitch = 0;
float derivPitch = 0;
float integPitch = 0;
float previousPitch = 0;

float errYaw = 0;
float derivYaw = 0;
float integYaw = 0;
float previousYaw = 0;

int powerA = 0;
int powerB = 0;
int powerC = 0;

unsigned long currentTime = millis();
unsigned long previousTime = millis();
unsigned long deltaT = 0;

float KP = 15;
float KI = 0;
float KD = 75;

void loop()
{
    if (MPU_N.read())
    {
        timer = millis();

        // Calculate Pitch, Roll and Yaw

        roll = MPU_N.m_fusedEulerPose[VEC3_X] * RAD_TO_DEGREE; // On récupère le roulis
        pitch = MPU_N.m_fusedEulerPose[VEC3_Y] * RAD_TO_DEGREE; // On récupère le tangage
        yaw = MPU_N.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE; // On récupère le lacet

        currentTime = millis();                 // On remet les variables temporelles en place
        deltaT = currentTime - previousTime;
        previousTime = currentTime;

        errRoll = roll;                          // On prépare les variables du PID
        derivRoll = errRoll - previousRoll;
        integRoll = integRoll + errRoll * deltaT;

        errPitch = pitch;
        derivPitch = errPitch - previousPitch;
        integPitch = integRoll + errPitch * deltaT;

        errYaw = yaw;
        derivYaw = errYaw - previousYaw;
        integYaw = integRoll + errYaw * deltaT;

        powerA = (KP*errRoll + KI*integRoll + KD*derivRoll);  // On calcule le PWM pour chaque moteur
        powerB = (KP*errPitch + KI*integPitch + KD*derivPitch + KP*errYaw + KI*integYaw + KD*derivYaw)/2.0;
        powerC = -(KP*errPitch + KI*integPitch + KD*derivPitch + KP*errYaw + KI*integYaw + KD*derivYaw)/2.0;

        previousRoll = roll;
        previousPitch = pitch;
        previousYaw = yaw;

        // Capper le PWM à 255
        if (powerA > 255) powerA = 255;
        else if (powerA < -255) powerA = -255;
        if (powerB > 255) powerB = 255;
        else if (powerB < -255) powerB = -255;
        if (powerC > 255) powerC = 255;
        else if (powerC < -255) powerC = -255;

        // Si le PWM est négatif, inverser la PIN_direction du moteur
        if (powerA >= 0) digitalWrite(PIN_DIR_A, HIGH);
        else
        {
            digitalWrite(PIN_DIR_A, LOW);
            powerA = -powerA;
        }
        if (powerB >= 0) digitalWrite(PIN_DIR_B, HIGH);
        else
        {
            digitalWrite(PIN_DIR_B, LOW);
            powerB = -powerB;
        }
        if (powerC >= 0) digitalWrite(PIN_DIR_C, HIGH);
        else
        {
            digitalWrite(PIN_DIR_C, LOW);
            powerC = -powerC;
        }

        // Arrêter le moteur si le cubli est tombé
        if (abs(roll) > 35.0 || abs(pitch) > 35.0 || abs(yaw) > 35.0)
        {
            analogWrite(PIN_PWM_A, 0);
            analogWrite(PIN_PWM_B, 0);
            analogWrite(PIN_PWM_C, 0);
        }
        Serial.println("");
    }
}
