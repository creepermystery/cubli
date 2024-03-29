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

void setup() {
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

float errRoll = 0.0;
float derivRoll = 0.0;
float deriv2Roll = 0.0;
float previousRoll = 0.0;
float previousDerivRoll = 0.0;
float targetRoll = 0.0;

float errPitch = 0.0;
float derivPitch = 0.0;
float deriv2Pitch = 0.0;
float previousPitch = 0.0;
float previousDerivPitch = 0.0;
float targetPitch = 0.0;

float errYaw = 0.0;
float derivYaw = 0.0;
float deriv2Yaw = 0.0;
float previousYaw = 0.0;
float previousDerivYaw = 0.0;
float targetYaw = 0.0;

int powerA = 0;
int powerB = 0;
int powerC = 0;

unsigned long currentTime = millis();
unsigned long previousTime = millis();
unsigned long deltaT = 0;

float angleFixRate = 0.0001; // 0.0001

float Kp = 10.5;
float Ki = 60;
float Kd = 3;

void loop() {
    if (MPU_N.read()) {

        timer = millis();

        // Calculate Pitch, Roll and Yaw

        roll = MPU_N.m_fusedEulerPose[VEC3_X] * RAD_TO_DEGREE; // On récupère le roulis
        pitch = MPU_N.m_fusedEulerPose[VEC3_Y] * RAD_TO_DEGREE; // On récupère le tangage
        yaw = MPU_N.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE; // On récupère le lacet

        currentTime = millis();                 // On remet les variables temporelles en place
        deltaT = currentTime - previousTime;
        previousTime = currentTime;

        errRoll = roll + targetRoll;
        derivRoll = (errRoll - previousRoll)/deltaT;
        deriv2Roll = (derivRoll - previousDerivRoll)/deltaT;
        
        errPitch = pitch + targetPitch;
        derivPitch = (errPitch - previousPitch)/deltaT;
        deriv2Pitch = (derivPitch - previousDerivPitch)/deltaT;
        
        errYaw = yaw + targetYaw;
        derivYaw = (errYaw - previousYaw)/deltaT;
        deriv2Yaw = (derivYaw - previousDerivYaw)/deltaT;

        powerA = (Kp*100.0*derivPitch + Ki/10.0*errPitch + Kd*3000.0*deriv2Pitch + Kp*100.0*derivYaw + Ki/10.0*errYaw + Kd*3000.0*deriv2Yaw)/2.0;
        powerB = powerA;
        powerC = powerA;

        float factorA = 0.0;
        float factorB = 0.0;
        float factorC = 0.0;

        if roll > -90 && roll < 90 {
            factorA += abs(1-(roll+90)/180);
            factorB -= abs(1-(roll+90)/180);
        } else {
            factorA -= abs(1-(roll+90)/180);
            factorB += abs(1-(roll+90)/180);
        }
        if roll > -30 && roll < 150 {
            factorA += abs(1-(roll+30)/180);
            factorC -= abs(1-(roll+30)/180);
        } else {
            factorA -= abs(1-(roll+30)/180);
            factorC += abs(1-(roll+30)/180);
        }
        if (roll > 30 && roll < 180) || (roll < -150 && roll > -180) {
            if roll > 30 && roll < 180 {
                factorB -= abs(1-(roll-30)/180);
                factorC -= abs(1-(roll-30)/180);
            } else {
                factorB -= abs(1-(roll+180)/180);
                factorC -= abs(1-(roll+180)/180);
            }
        } else {
            if roll < 30 && roll > -180 {
                factorB += abs(1-(roll-30)/180);
                factorC += abs(1-(roll-30)/180);
            } else {
                factorB += abs(1-(roll+180)/180);
                factorC += abs(1-(roll+180)/180);
            }
        }

        powerA *= factorA;
        powerB *= factorB;
        powerC *= factorC;

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
        else {
            powerA = -powerA;
            digitalWrite(PIN_DIR_A, LOW);
        }
        if (powerB >= 0) digitalWrite(PIN_DIR_B, HIGH);
        else {
            powerB = -powerB;
            digitalWrite(PIN_DIR_B, LOW);
        }
        if (powerC >= 0) digitalWrite(PIN_DIR_C, HIGH);
        else {
            powerC = -powerC;
            digitalWrite(PIN_DIR_C, LOW);
        }

        // Arrêter le moteur si le cubli est tombé
        if (abs(pitch) > 45.0 || abs(yaw) > 45.0) {
            analogWrite(PIN_PWM_A, 0);
            analogWrite(PIN_PWM_B, 0);
            analogWrite(PIN_PWM_C, 0);
        }

        if (errRoll < 0) targetRoll += angleFixRate*deltaT;
        else targetRoll -= angleFixRate*deltaT;
        if (errPitch < 0) targetPitch += angleFixRate*deltaT;
        else targetPitch -= angleFixRate*deltaT;
        if (errYaw < 0) targetYaw += angleFixRate*deltaT;
        else targetYaw -= angleFixRate*deltaT;
    }
}
