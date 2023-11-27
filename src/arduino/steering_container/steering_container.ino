//-------------------------------------------------------//
// Library
#include "BTS7960.h"
#include <SoftPWM.h>

//-------------------------------------------------------//
// pin and constant
//** Front Steer
const uint8_t EN_FRONT_STEER = 3;
const uint8_t L_FRONT_STEER = 2;
const uint8_t R_FRONT_STEER = 4;
BTS7960 Front_Steer(EN_FRONT_STEER, L_FRONT_STEER, R_FRONT_STEER);
//** Rear Steer
const uint8_t EN_REAR_STEER = 6;
const uint8_t L_REAR_STEER = 5;
const uint8_t R_REAR_STEER = 7;
BTS7960 Rear_Steer(EN_REAR_STEER, L_REAR_STEER, R_FRONT_STEER);
//** Braking System
const uint8_t EN_LOCK = 9;
const uint8_t L_LOCK = 8;
const uint8_t R_LOCK = 13; //notes, ditukar
BTS7960 Lock(EN_LOCK, L_LOCK, R_LOCK);
//** Steer State
/* notes :
   0 = front wheel steer,
   1 = rear wheel steer,
   2 = all wheel steer.
*/
//** Brake State
bool brake_now = false;
bool brake_bef = true;


// Define digital pin from Arduino Mega
#define MEGA1 A0
#define MEGA2 A1
#define MEGA3 A2
#define MEGA4 A3
#define MEGA5 A4

// Variables
byte wheel_steer_mode = 0;
byte steer_increment = 0;
byte container_lock = 0;


void setup() {
  // put your setup code here, to run once:
  // Declare output pin
  for (int i = 13; i > 1; i--)
  {
    pinMode(i, OUTPUT);
  }

  // Software PWM Setting
  SoftPWMBegin();

  // Read digital pin from Arduino Mega
  pinMode(MEGA1, INPUT_PULLUP);
  pinMode(MEGA2, INPUT_PULLUP);
  pinMode(MEGA3, INPUT_PULLUP);
  pinMode(MEGA4, INPUT_PULLUP);
  pinMode(MEGA5, INPUT_PULLUP);

  // Serial monitor
  Serial.begin(9600);
  Serial.println("READY!");
}


//////////// MAIN LOOP
void loop() {
  // put your main code here, to run repeatedly:

  if (!digitalRead(MEGA1)) {
    wheel_steer_mode = 0; // Front only
  } else if (!digitalRead(MEGA2)) {
    wheel_steer_mode = 1; // Rear only
  } else {
    wheel_steer_mode = 2; // All
  }

  if (!digitalRead(MEGA3)) {
    steer_increment = 0;  // Left increment
  } else if (!digitalRead(MEGA4)) {
    steer_increment = 1;  // Right increment
  } else {
    steer_increment = 2;  // No increment
  }

  if (!digitalRead(MEGA5)) {
    brake_now = 1;
  } else {
    brake_now = 0;
  }


  action_steer(wheel_steer_mode, steer_increment);
  action_container_lock();

}


//////////// ADDED FUNCTION

void action_steer(byte value, byte increment)
{
  if (increment == 1) //(!digitalRead(A3)) // Increment Left
  {
    if (value == 0)
    {
      Front_Steer.Enable();
      SoftPWMSet(L_FRONT_STEER, 255);
      SoftPWMSet(R_FRONT_STEER, 0);
      Serial.println("Turn Right FW");
    }
    else if (value == 1)
    {
      Rear_Steer.Enable();
      SoftPWMSet(L_REAR_STEER, 0);
      SoftPWMSet(R_REAR_STEER, 255);
      Serial.println("Turn Right RW");
    }
    else if (value == 2)
    {
      Front_Steer.Enable();
      Rear_Steer.Enable();
      SoftPWMSet(L_FRONT_STEER, 255);
      SoftPWMSet(R_REAR_STEER, 255);
      Serial.println("Turn Right AW");
    }
  }
  else if (increment == 0) //(!digitalRead(A4)) // Increment right
  {
    if (value == 0)
    {
      Front_Steer.Enable();
      SoftPWMSet(R_FRONT_STEER, 255);
      SoftPWMSet(L_FRONT_STEER, 0);
      Serial.println("Turn Left FW");
    }
    else if (value == 1)
    {
      Rear_Steer.Enable();
      SoftPWMSet(R_FRONT_STEER, 0);
      SoftPWMSet(L_REAR_STEER, 255);
      Serial.println("Turn Left RW");
    }
    else if (value == 2)
    {
      Front_Steer.Enable();
      Rear_Steer.Enable();
      SoftPWMSet(R_FRONT_STEER, 255);
      SoftPWMSet(L_REAR_STEER, 255);
      Serial.println("Turn Left AW");
    }
  }
  else
  {
    //do nothing
    SoftPWMSet(R_REAR_STEER, 0);
    SoftPWMSet(L_REAR_STEER, 0);
    SoftPWMSet(L_FRONT_STEER, 0);
    SoftPWMSet(R_FRONT_STEER, 0);
    Front_Steer.Stop();
    Rear_Steer.Stop();
  }
}

void action_container_lock()
{
  // Running hanya saat berubah
  if (brake_now != brake_bef)
  {
    if (brake_now)
    {
      Lock.Enable();
      Serial.println("engaging brake");
      SoftPWMSet(L_LOCK, 255);
      delay(3000);
      Serial.println("Done");
      brake_bef = brake_now;
    }
    else if (!brake_now)
    {
      Lock.Enable();
      Serial.println("disengaging brake");
      brake_bef = brake_now;
      SoftPWMSet(R_LOCK, 255);
      delay(3000);
      Serial.println("Done");
    }
  }
  else
  {
    SoftPWMSet(L_LOCK, 0);
    SoftPWMSet(R_LOCK, 0);
    Lock.Stop();
  }
}
