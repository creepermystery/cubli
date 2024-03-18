

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

float err = 0.0;
float dterr = 0.0;
float dtdterr = 0.0;
float prev_dterr = 0.0;

int power = 0;

int PowerSature = 0;
float PowerIntegral = 0;


unsigned long currentTime = millis();
unsigned long previousTime = millis();
unsigned long deltaT = 0;
float Angle_Fixrate=0.0001;                 //0.0001

float Kp = 10.5;    //  20.72   22.25   16    10.5  10.5
float Ki = 60;      //  18.25   21      60    60    60
float Kd = 3;       //  14.0    18      0     1     3
float Kt = 0;       // anglefixerate
float Target_angle = 8.5;

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

        
    if(err<0){
        Target_angle+=Angle_Fixrate*deltaT;
    }
    else{
        Target_angle-=Angle_Fixrate*deltaT;
    }
    
    
    err = roll + Target_angle;           //qualibration mpu    
               // On prépare les variables du PID
    dterr = (err - previousroll)/deltaT;
    dtdterr = (dterr-prev_dterr)/deltaT;

    //PowerIntegral = PowerIntegral+(Ki/10*err + Kt*(PowerSature - power) )/deltaT;
    //power = (Kp*100*dterr + PowerIntegral + Kd*3000*dtdterr);  // On calcule le PWM
    power = (Kp*100*dterr + Ki/10*err + Kd*3000*dtdterr);
    previousroll = err;
    prev_dterr = dterr;
    
    // Capper le PWM à 255
   /* if (power > 255) PowerSature = 255;
    else if (power < -255) PowerSature = -255;
    else power = PowerSature;*/
    if (power > 255) power = 255;
    else if (power < -255) power = -255;

    // Si le PWM est négatif, inverser la PIN_direction du moteur
    if (power >= 0) digitalWrite(PIN_DIR, LOW);
    else
    {
        digitalWrite(PIN_DIR, HIGH);
        power = -power;
    }


    // Arrêter le moteur si le cubli est tombé
    if ((roll+8.5 > -45.0) && (roll+8.5 < 45.0)) analogWrite(PIN_PWM, power);
    else analogWrite(PIN_PWM, 0);
    
    
    /*Serial.print(err);
    Serial.print("\t");
    Serial.print(dterr*Kp*100);
    Serial.print("\t");
    Serial.print(err*Ki/10);
    Serial.print("\t");
    Serial.print(dtdterr*Kd*3000);
    Serial.print("\t");
    Serial.print(power);
    Serial.println("");
    */
    }
}
