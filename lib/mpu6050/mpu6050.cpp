#include <Arduino.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>
#include "mpu6050.h"
#include "..\..\include\common.h"

void init_mpu_gyro()
{
    Wire.begin();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    //gyro config
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x10);
    Wire.endTransmission(true);
    //accelometer config
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission(true);

    current_gyro_time = millis();

    if (accel_offset[ROLL] == 0) {
    for (int i = 0; i < 200; i++) {

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, true);

      x_accel = -(Wire.read() << 8 | Wire.read()) / 4096.0 ;
      y_accel = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      z_accel = -(Wire.read() << 8 | Wire.read()) / 4096.0 ;

      accel_offset[PITCH] += (atan((y_accel) / sqrt(pow((x_accel), 2) + pow((z_accel), 2))) * RAD_TO_DEG);
      accel_offset[ROLL] += (atan(-1 * (x_accel) / sqrt(pow((y_accel), 2) + pow((z_accel), 2))) * RAD_TO_DEG);

      if (i == 199) {
        accel_offset[ROLL] = accel_offset[ROLL] /  200;
        accel_offset[PITCH] = accel_offset[PITCH] / 200;
      }
    }
  }
  if (gyro_offset[ROLL] == 0) {
    for (int i = 0; i < 200; i++) {

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, true);

      x_gyro = -(Wire.read() << 8 | Wire.read());
      y_gyro = Wire.read() << 8 | Wire.read();
      z_gyro = -(Wire.read() << 8 | Wire.read());

      gyro_offset[ROLL] += y_gyro / 32.8 ;
      gyro_offset[PITCH] += x_gyro / 32.8;
      gyro_offset[YAW] += z_gyro / 32.8;
      if (i == 199) {
        gyro_offset[ROLL] = gyro_offset[ROLL] / 200;
        gyro_offset[PITCH] = gyro_offset[PITCH] / 200;
        gyro_offset[YAW] = gyro_offset[YAW] / 200;
      }
    }
  }
}

void get_mpu_data(double *mpu_current_data)
{
    previous_gyro_time = current_gyro_time;
    current_gyro_time = millis();
    change_in_gyro_time = (current_gyro_time - previous_gyro_time) / 1000;

    /*gyroscope*/
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    x_gyro = -(Wire.read() << 8 | Wire.read());
    y_gyro = Wire.read() << 8 | Wire.read();
    z_gyro = -(Wire.read() << 8 | Wire.read());

    x_gyro = (x_gyro / 32.8) - gyro_offset[PITCH];
    y_gyro = (y_gyro / 32.8) - gyro_offset[ROLL];
    z_gyro = (z_gyro / 32.8) - gyro_offset[YAW];

    double pitch_gyro = x_gyro * change_in_gyro_time;
    double roll_gyro = y_gyro * change_in_gyro_time;
    double yaw_gyro = z_gyro * change_in_gyro_time;

    /*accelerometer*/
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    x_accel = -(Wire.read() << 8 | Wire.read()) / 4096.0 ;
    y_accel = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    z_accel = -(Wire.read() << 8 | Wire.read()) / 4096.0 ;

    double pitch_accel = (atan((y_accel) / sqrt(pow((x_accel), 2) + pow((z_accel), 2))) * RAD_TO_DEG) - accel_offset[PITCH];
    double roll_accel = (atan(-1 * (x_accel) / sqrt(pow((y_accel), 2) + pow((z_accel), 2))) * RAD_TO_DEG) - accel_offset[ROLL];

    /*Filter*/
    mpu_current_data[ROLL] = 0.98 * (mpu_current_data[ROLL] + roll_gyro) + 0.02 * (roll_accel);
    mpu_current_data[PITCH] = 0.98 * (mpu_current_data[PITCH] + pitch_gyro) + 0.02 * (pitch_accel);
    double yaw_accel = atan2((sin(mpu_current_data[ROLL]) * cos(mpu_current_data[PITCH]) * x_accel + sin(mpu_current_data[PITCH]) * y_accel + cos(mpu_current_data[ROLL]) * cos(mpu_current_data[PITCH]) * z_accel), sqrt(pow(sin(mpu_current_data[ROLL]) * sin(mpu_current_data[PITCH]) * x_accel - cos(mpu_current_data[ROLL]) * sin(mpu_current_data[PITCH]) * z_accel, 2) + pow(cos(mpu_current_data[PITCH]) * x_accel, 2))) - 1;
    mpu_current_data[YAW] = 0.98 * (mpu_current_data[YAW] + yaw_gyro) + 0.02 * (yaw_accel);
}

/*TODO: Temperature*/
double get_temp()
{
}