

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
bool newRoll = false;

int pos = 1500;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(1000);
    MPU_N.init(MPU_UPDATE_RATE); 
   
    // configure outputs
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
}

float previousRoll = 0;

float err = 0.0;
float dtErr = 0.0;
float dtDtErr = 0.0;
float previousDtErr = 0.0;

int power = 0;

unsigned long currentTime = millis();
unsigned long previousTime = millis();
unsigned long deltaT = 0;

float angleFixRate = 0.0001; // 0.0001

float Kp = 10.5;
float Ki = 60;
float Kd = 3;
// float Kt = 0;
float targetAngle = 0.0;

void loop() {
    if (MPU_N.read()) {

        timer = millis();

        // Calculate Pitch, Roll and Yaw

        roll = MPU_N.m_fusedEulerPose[VEC3_X] * RAD_TO_DEGREE;
        pitch = MPU_N.m_fusedEulerPose[VEC3_Y] * RAD_TO_DEGREE;
        yaw = MPU_N.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE;// On récupère le lacet
    
        currentTime = millis();                 // On remet les variables temporelles en place
        deltaT = currentTime - previousTime;
        previousTime = currentTime;
        
        // On prépare les variables du PID
        err = roll + targetAngle; // calibration mpu    
        dtErr = (err - previousRoll)/deltaT;
        dtDtErr = (dtErr-previousDtErr)/deltaT;

        //PowerIntegral = PowerIntegral+(Ki/10*err + Kt*(PowerSature - power) )/deltaT;
        //power = (Kp*100*dtErr + PowerIntegral + Kd*3000*dtDtErr);  // On calcule le PWM
        power = (Kp*100*dtErr + Ki/10*err + Kd*3000*dtDtErr);
        previousRoll = err + targetAngle;
        previousDtErr = dtErr;
        
        // Capper le PWM à 255
        /* if (power > 255) PowerSature = 255;
        else if (power < -255) PowerSature = -255;
        else power = PowerSature;*/
        if (power > 255) power = 255;
        else if (power < -255) power = -255;

        // Si le PWM est négatif, inverser la PIN_direction du moteur
        if (power >= 0) digitalWrite(PIN_DIR, LOW);
        else {
            digitalWrite(PIN_DIR, HIGH);
            power = -power;
        }

        // Arrêter le moteur si le cubli est tombé
        if ((roll > -45.0) && (roll < 45.0)) analogWrite(PIN_PWM, power);
        else analogWrite(PIN_PWM, 0);

        if (err<0) {
            targetAngle += angleFixRate*deltaT;
        } else {
            targetAngle -= angleFixRate*deltaT;
        }
    }
}
