#ifndef PARAMS_H
#define PARAMS_H

#define RUNNING_AVG_SIZE 20

// Bluetooth pins
#define REQ_PIN 30    // SPI select pin
#define RDY_PIN 18    // Sends interrupt to MCU when data is ready (Needs to be an interrupt pin)
#define RST_PIN 32    // Resets the bluetooth board (Used during startup)

// SpineAligner on/off
#define ON 1
#define OFF 0

// motor pins
#define UPPER_MOTOR 28
#define LOWER_MOTOR 29

// number of sensors on vest
#define NUM_IMUS 3

// state of spine
#define VERY_GOOD 0
#define GOOD 1
#define BAD 2
#define VERY_BAD 3

// form of spine
#define STRAIGHT 0
#define LOWER_ARCH -1
#define BOTH_ARCH -2
#define UPPER_HUNCH 1
#define LOWER_HUNCH 3
#define BOTH_HUNCH 2

// z angle thresholds
#define DEG20 -1
#define DEG45 -3.5
#define DEG90 -9

// loop threshold value
#define T_VAL 25

// change amounts for 20 degree zone
#define TMP_LA20 -1.5
#define TMP_UH20 1.2
#define TMP_BH20 1.2

// sensitivity
#define INCR_DECR 0.2
#define MAX_CHANGE 3

#endif
