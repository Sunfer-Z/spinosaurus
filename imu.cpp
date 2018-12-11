#include "imu.h"
#include "MPU9250.h"
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include "Arduino.h"
#define CAL_THRESH 100

/* --------------------------- Setup Functions --------------------------- */

/* Initializes data collection for a particular IMU. Tells the IMU what data it should be sampling 
 * and placing in its buffer.
 */
IMU_T::IMU_T(int imu_id_arg = 0) {
    imu_id = imu_id_arg;
    accel_index = 0;
    //gyro_index = 0;
    //mag_index = 0;
}

void IMU_T::init_pin(int pin_num) {
  AD0_pin = pin_num;
  pinMode(AD0_pin, OUTPUT);
}

/* Sets the full scale of the accelerometer values:
 * 0 = ±2g
 * 1 = ±4g
 * 2 = ±8g
 * 3 = ±16g
 */
void IMU_T::set_accel_scale(MPU9250 &imu_device, int val) {
  switch(val) {
    case 0:
      imu_device.setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_2G);
      break;
    case 1:
      imu_device.setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_4G);
      break;
    case 2:
      imu_device.setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_8G);
      break;
    case 3:
      imu_device.setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_16G);
      break;
    default:
      Serial.print("Please input a valid ACCEL_RANGE value\n");
      Serial.print("0 = +/-2G, 1 = +/-4G, 2 = +/-8G, 3 = +/-16G\n");
      return;
  }
}

/* Sets the full scale of gyroscope values:
 * 0 = ±250dps
 * 1 = ±500dps 
 * 2 = ±1000dps 
 * 3 = ±2000dps
 */
void IMU_T::set_gyro_scale(MPU9250 &imu_device, int val) {
  switch(val) {
    case 0:
      imu_device.setGyroRange(MPU9250::GyroRange::GYRO_RANGE_250DPS);
      break;
    case 1:
      imu_device.setGyroRange(MPU9250::GyroRange::GYRO_RANGE_500DPS);
      break;
    case 2:
      imu_device.setGyroRange(MPU9250::GyroRange::GYRO_RANGE_1000DPS);
      break;
    case 3:
      imu_device.setGyroRange(MPU9250::GyroRange::GYRO_RANGE_2000DPS);
      break;
    default:
      Serial.print("Please input a valid GYRO_RANGE value\n");
      Serial.print("0 = +/-2G, 1 = +/-4G, 2 = +/-8G, 3 = +/-16G\n");
      return;
  } 
}

/* --------------------------- Data Functions --------------------------- */

/* Gets accelerometer data axis x from I2C line. */
float IMU_T::get_accel_data_x(MPU9250 &imu_device) {
    imu_device.readSensor();
    return imu_device.getAccelX_mss();
}

/* Gets accelerometer data axis y from I2C line. */
float IMU_T::get_accel_data_y(MPU9250 &imu_device) {
    imu_device.readSensor();
    return imu_device.getAccelY_mss();
}

/* Gets accelerometer data axis z from I2C line. */
float IMU_T::get_accel_data_z(MPU9250 &imu_device) {
    imu_device.readSensor();
    return imu_device.getAccelZ_mss();
}

/* --------------------------- Calibration Functions --------------------------- */

/* Allows initial calibration with threshold for calibration allowance specified in 
 * percentage or degrees of freedom.
 */
void IMU_T::calibrate_IMU(MPU9250 &imu_device, int angle) {
  float x_accel_sum = 0;
  float y_accel_sum = 0;
  float z_accel_sum = 0;

  // get avg over a small number of readings
  for (int i = 0; i < CAL_THRESH; ++i) {
    imu_device.readSensor();
    x_accel_sum = x_accel_sum + imu_device.getAccelX_mss();
    y_accel_sum = y_accel_sum + imu_device.getAccelY_mss();
    z_accel_sum = z_accel_sum + imu_device.getAccelZ_mss();
  }
  
  accel_xyz[0].offset[angle] = (x_accel_sum / CAL_THRESH);
  accel_xyz[1].offset[angle] = (y_accel_sum / CAL_THRESH);
  accel_xyz[2].offset[angle] = (z_accel_sum / CAL_THRESH);
}

void IMU_T::update_accel_averages(MPU9250 &imu_device) {
  float data[3];
  imu_device.readSensor();
  data[0] = imu_device.getAccelX_mss();
  data[1] = imu_device.getAccelY_mss();
  data[2] = imu_device.getAccelZ_mss();
  for(int i = 0; i < 3; ++i) {
    accel_xyz[i].total = accel_xyz[i].total - accel_xyz[i].readings[accel_index];
    accel_xyz[i].readings[accel_index] = data[i];
    accel_xyz[i].total += accel_xyz[i].readings[accel_index];
    accel_xyz[i].avg = accel_xyz[i].total / RUNNING_AVG_SIZE;
  }
}
