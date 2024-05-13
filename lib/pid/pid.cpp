#include <Arduino.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "radio.h"
#include "..\..\include\common.h"
#include "Servo.h"

void get_setpoints(double *set_point, Controller_Data_Message *msgbuf)
{
    set_point[YAW] = msgbuf->j1PotX - 128;
    Serial.println(set_point[YAW]);
    set_point[ROLL] = msgbuf->j2PotX - 128;
    set_point[PITCH] = msgbuf->j2PotY - 128;
    set_point[THRUST] = msgbuf->j1PotY - 128;  
}

/*PID Formula/Algorithm:
output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
e(t) -> error function (setpoint - current)
Kp   -> tuning parameter: base error function
Ki   -> tuning parameter: intergated error function
Kd   -> tuning parameter: derivied error function
interval -> time interval between call to calculate pid function, it is const here
*/
void calculate_pid(double *set_point, double *current, double *last_error, double *integral, double *output) 
{
    double error[3] = {
        set_point[YAW] - current[YAW],
        set_point[ROLL] - current[ROLL],
        set_point[PITCH] - current[PITCH]
    };

    last_error[YAW] = error[YAW];
    last_error[ROLL] = error[ROLL];
    last_error[PITCH] = error[PITCH];

    integral[YAW] += TIME_INTERVAL * error[YAW];
    integral[ROLL] += TIME_INTERVAL * error[ROLL];
    integral[PITCH] += TIME_INTERVAL * error[PITCH];

    double derivative[3] = {
        (error[YAW] - last_error[YAW]) / TIME_INTERVAL,
        (error[ROLL] - last_error[ROLL]) / TIME_INTERVAL,
        (error[PITCH] - last_error[PITCH]) / TIME_INTERVAL
    };

    output[YAW] = Kp[YAW] * error[YAW] + Ki[YAW] * integral[YAW] + Kd[YAW] * derivative[YAW];
    output[ROLL] = Kp[ROLL] * error[ROLL] + Ki[ROLL] * integral[ROLL] + Kd[ROLL] * derivative[ROLL];
    output[PITCH] = Kp[PITCH] * error[PITCH] + Ki[PITCH] * integral[PITCH] + Kd[PITCH] * derivative[PITCH];
    output[THRUST] = set_point[THRUST];
}

void pid_motors(double *pid_output, Servo &servo_fl, Servo &servo_fr, Servo &servo_rl, Servo &servo_rr)
{
    /*Always have thrust on, but then there are three other possibilities: roll, pitch or yaw
    for roll, it turns on left or right. Add output on left as if it is large it will turn right
    small it turns left. For pitch*/
    double speed_fl = pid_output[YAW] + pid_output[ROLL] + pid_output[PITCH] + pid_output[THRUST];
    double speed_fr = - pid_output[YAW] - pid_output[ROLL] + pid_output[PITCH] + pid_output[THRUST];
    double speed_rl = - pid_output[YAW] + pid_output[ROLL] - pid_output[PITCH] + pid_output[THRUST];
    double speed_rr = pid_output[YAW] - pid_output[ROLL] - pid_output[PITCH] + pid_output[THRUST];

    /*speed_fl = constrain(speed_fl, 0, 360) / 2;
    speed_fr = constrain(speed_fr, 0, 360) / 2;
    speed_rl = constrain(speed_rl, 0, 360) / 2;
    speed_rr = constrain(speed_rr, 0, 360) / 2;*/

    speed_fl = constrain(speed_fl, -128, 127);
    speed_fr = constrain(speed_fr, -128, 127);
    speed_rl = constrain(speed_rl, -128, 127);
    speed_rr = constrain(speed_rr, -128, 127);

    speed_fl = map(speed_fl, -128, 127, 1000, 2000);
    speed_fr = map(speed_fr, -128, 127, 1000, 2000);
    speed_rl = map(speed_rl, -128, 127, 1000, 2000);
    speed_rr = map(speed_rr, -128, 127, 1000, 2000);

    if (TESTING){
        Serial.print("front left");
        Serial.println(speed_fl);
        Serial.print("front right");
        Serial.println(speed_fr);
        Serial.print("back left");
        Serial.println(speed_rl);
        Serial.print("back right");
        Serial.println(speed_rr);
        delay(1000);
    }

    if (!TESTING){
        /*TODO: write to esc*/
        servo_fl.writeMicroseconds(speed_fl);
        servo_fr.writeMicroseconds(speed_fr);
        servo_rl.writeMicroseconds(speed_rl);
        servo_rr.writeMicroseconds(speed_rr);
        delay(1);
    }
}