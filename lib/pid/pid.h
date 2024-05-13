/*pid.h*/
#ifndef DRONE_PID_H_GUARD
#define DRONE_PID_H_GUARD

#include "..\..\include\common.h"
#include "Servo.h"

/*Function Prototypes*/
void get_setpoints(double *set_point, Controller_Data_Message *msgbuf);
void calculate_pid(double *set_point, double *current, double *last_error, double *integral, double *output);
void pid_motors(double *pid_output, Servo &servo_fl, Servo &servo_fr, Servo &servo_rl, Servo &servo_rr);

#endif