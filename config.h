/*
 * config.h
 *
 *  Created on: Aug 1, 2012
 *      Author: dart
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* Multicopter Type */
#if !defined(SINGLE_COPTER) \
    && !defined(DUAL_COPTER) \
    && !defined(TWIN_COPTER) \
    && !defined(TRI_COPTER) \
    && !defined(QUAD_COPTER) \
    && !defined(QUAD_X_COPTER) \
    && !defined(Y4_COPTER) \
    && !defined(HEX_COPTER) \
    && !defined(Y6_COPTER)
//#define SINGLE_COPTER
//#define DUAL_COPTER
//#define TWIN_COPTER
//#define TRI_COPTER
//#define QUAD_COPTER
#define QUAD_X_COPTER
//#define Y4_COPTER
//#define HEX_COPTER
//#define Y6_COPTER
#endif


//#define GAIN_POT_REVERSE

// Gyro gain shift-right (after 32-bit multiplication of GYRO_GAIN_ADC_S value).
#define GYRO_GAIN_SHIFT 5

// Skip yaw gyro calculations if using external yaw gyro
//#define EXTERNAL_YAW_GYRO

//#define SERVO_REVERSE

/*
 * ESC PPM output rate -
 * Do not set lower than 122 Hz without a slower/higher t0/t1 clkdiv
 * as the period needs to fit in the 16-bit timer.
 */
//#define ESC_RATE 300  // in Hz
//#define ESC_RATE 400  // in Hz (at SINGLE_COPTER Only)
#define ESC_RATE 450  // in Hz
//#define ESC_RATE 495  // in Hz

// NOTE: Set to 50 for analog servos, 250 for digital servos.
#define SERVO_RATE 50  // in Hz

// Stick arming and throw detection (in % * 10 eg 1000 steps)
#define STICK_THROW 300

// Stick gain shift-right (after 32-bit multiplication of GYRO_GAIN_ADC_S value).
#define STICK_GAIN_SHIFT 8

// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilise better when full throttle applied
#define MAX_COLLECTIVE 1000      // 95

// Lowest values at which motor is still rotating
#define MOTOR_LOWEST_VALUE  114

#endif /* CONFIG_H_ */
