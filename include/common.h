//common.h
#ifndef DRONE_COMMON_H_GUARD
#define DRONE_COMMON_H_GUARD

#include <Arduino.h>

// Definition of the Controller Message Struct
struct Controller_Data_Message {
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte button1;
  byte button2;
  byte buzzer;
  byte batteryLevel;
  byte test;
};

// Definition of the Drone Message Struct
struct Drone_Data_Message {
  byte temp;
  byte pressure;
  byte gpsLatitude;
  byte gpsLongitude;
  byte batteryLevel;
  byte location;
  byte speed;
};

/*MPU Address Table*/
#define MPU_ADDR 0x68

/*pin definitions*/
#define ESC_PIN_FRONT_LEFT 8 /*ESC1*/
#define ESC_PIN_FRONT_RIGHT 9 /*ESC2*/
#define ESC_PIN_REAR_LEFT 4 /*ESC3*/
#define ESC_PIN_REAR_RIGHT 3 /*ESC4*/

#define NRF_CE A9
#define NRF_CSN A10

#define GPS_RX 6
#define GPS_TX 7

/*Threshold constants*/
const uint16_t MOTOR_OFF = 1000;
const uint8_t TIME_INTERVAL = 40;

/*Movement type constants*/
#define YAW 1
#define ROLL 2
#define PITCH 3
#define THRUST 4 

/*Propotional Constants*/
const double Kp[] = {0.8, 0.8, 0.12};
const double Ki[] = {0.0000002, 0.0000002, 0.0000005};
const double Kd[] = {0.05, 0.05, 0.052};

/*Testing variable*/
extern bool TESTING;

extern double current_gyro_time, previous_gyro_time, change_in_gyro_time;
extern double current_time, previous_time;
extern double x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro;
extern double accel_offset[3], gyro_offset[3]; 

#endif