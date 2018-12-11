#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"
#include "params.h"

struct FSM {
  int imu_bottom, imu_top;
  int state, form;
  float BA[3], LA[3], UH[3], BH[3];
  int THRE_GOOD, THRE_BAD;
  int count_g, count_b;
};

struct AxisVals {
  float readings[RUNNING_AVG_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float total = 0;
  float avg = 0;
  float offset[3] = {0.0, 0.0, 0.0};
};

class IMU_T {
public:
    int AD0_pin;    // LSB of slave address of the IMU
    int imu_id;     // unique id of each IMU
    int accel_index;
    AxisVals accel_xyz[3];
    
    /* --------------------------- Setup Functions --------------------------- */
    
    /* Constructor. Initializes data collection for a particular IMU.
     * Tells the IMU what data it should be sampling and placing in its buffer.
     */
    IMU_T(int imu_id_arg = 0);

    void init_pin(int pin_num);

    /* Sets the full scale of the accelerometer values:
     * 0 = ±2g
     * 1 = ±4g
     * 2 = ±8g
     * 3 = ±16g
     */
    void set_accel_scale(MPU9250 &imu_device, int val);
    
    /* Sets the full scale of gyroscope values:
     * 0 = ±250dps
     * 1 = ±500dps
     * 2 = ±1000dps
     * 3 = ±2000dps
     */
    void set_gyro_scale(MPU9250 &imu_device, int val);
    
    /* --------------------------- Data Functions --------------------------- */
    
    /* Gets accelerometer data axis x from I2C line. */
    float get_accel_data_x(MPU9250 &imu_device);
    
    /* Gets accelerometer data axis y from I2C line. */
    float get_accel_data_y(MPU9250 &imu_device);
    
    /* Gets accelerometer data axis z from I2C line. */
    float get_accel_data_z(MPU9250 &imu_device);
    
    /* --------------------------- Calibration Functions --------------------------- */
    
    /* Allows initial calibration with threshold for calibration allowance specified in
     * percentage or degrees of freedom.
     */
    void calibrate_IMU(MPU9250 &imu_device, int angle);

    void update_accel_averages(MPU9250 &imu_device);
};

#endif
