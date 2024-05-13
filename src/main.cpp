#include <Arduino.h>
#include <radio.h>
#include <setup_init.h>
#include "common.h"
#include "mpu6050.h"
#include "pid.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"

/*Initiate tx and rx messages*/
Controller_Data_Message rx_msg;
Drone_Data_Message tx_msg;

double set_points[4] = {0,0,0,0};
double current_points[4] = {0,0,0,0};
double last_error[3] = {0,0,0};
double integral[3] = {0,0,0};
double outputs[4] = {0,0,0};

double current_gyro_time = 0.0;
double previous_gyro_time = 0.0;
double change_in_gyro_time = 0.0;
double current_time = 0.0;
double previous_time = 0.0;
double x_accel = 0.0, y_accel = 0.0, z_accel = 0.0;
double x_gyro = 0.0, y_gyro = 0.0, z_gyro = 0.0;
double accel_offset[3] = {0.0};
double gyro_offset[3] = {0.0};

RF24 radio(NRF_CE, NRF_CSN);   
SoftwareSerial gps_serial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

bool TESTING = true;

Servo esc_fl;
Servo esc_fr;
Servo esc_rl;
Servo esc_rr;

// put function declarations here:
void init_serial();
void init_pins();
void init_spi();
void init_mpu_gyro();
void rx_spi(Controller_Data_Message *msgbuf);

void setup() {
  // put your setup code here, to run once:
  init_serial();
  //init_pins();
  gps_serial.begin(9600);

  esc_fl.attach(ESC_PIN_FRONT_LEFT);
  esc_fr.attach(ESC_PIN_FRONT_RIGHT);
  esc_rl.attach(ESC_PIN_REAR_LEFT);
  esc_rr.attach(ESC_PIN_REAR_RIGHT);
  
  esc_fl.writeMicroseconds(1000);
  esc_fr.writeMicroseconds(1000);
  esc_rl.writeMicroseconds(1000);
  esc_rr.writeMicroseconds(1000);
  
  delay(2000);  // wait for the ESCs to calibrate
  init_spi();
  init_mpu_gyro();
}

void loop() {
  // put your main code here, to run repeatedly:
  rx_spi(&rx_msg);

  get_setpoints(set_points, &rx_msg);
  get_mpu_data(current_points);

  current_time = millis();
  if ((current_time - previous_time) > TIME_INTERVAL){
    previous_time = current_time;
    calculate_pid(set_points, current_points, last_error, integral, outputs);
  }

  pid_motors(outputs, esc_fl, esc_fr, esc_rl, esc_rr);
  while (gps_serial.available() > 0){
    gps.encode(gps_serial.read());
    tx_msg.gpsLatitude = gps.;
    tx_msg.speed = gps.speed;
    delay(1000);
  }
  //pid_motors(set_points, esc_fl, esc_fr, esc_rl, esc_rr);
  /*TODO: Change to Microsecond pulses*/

  tx_spi(&tx_msg);
}

/*TODO: implement GPS, LED Peripherals, buzzer*/

// put function definitions here:
