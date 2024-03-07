/*
 le fichier I2Cdev.cpp a été modifié a la ligne 193 avec l'ajout de :
#define min(a,b) ((a)<(b)?(a):(b))
Pour éviter une erreur compil
 */

#include <Wire.h>
#include "MPU9150Lib.h"

#define MPU_UPDATE_RATE  (10)

MPU9150Lib MPU_N; 
float yaw, pitch, roll;
float accRoll;
float accRollInt;
float rollPlusUn;
bool newRoll = false;

int pos = 1500;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Wire.setSDA(D4);
  //Wire.setSCL(D5);
  Wire.begin();
  delay(1000);
  MPU_N.init(MPU_UPDATE_RATE);   
}

void loop() {
  // put your main code here, to run repeatedly:
  if (MPU_N.read())
  { 
    roll = MPU_N.m_fusedEulerPose[VEC3_X] * RAD_TO_DEGREE;
    pitch = MPU_N.m_fusedEulerPose[VEC3_Y] * RAD_TO_DEGREE;
    yaw = MPU_N.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE;
    //roll = roll + 6;
    Serial.print("roll=");
    Serial.print(roll);
    Serial.print("\tYaw=");
    Serial.print(yaw);
    Serial.print("\tPitch=");
    Serial.print(pitch);
    Serial.println();
  }
}

