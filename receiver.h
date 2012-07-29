#ifndef RECEIVER_H
#define RECEIVER_H

#include "config.h"

/*** BEGIN DEFINES ***/
// Stick arming and throw detection (in % * 10 eg 1000 steps)
#define STICK_THROW 300

// Stick gain shift-right (after 32-bit multiplication of GYRO_GAIN_ADC_S value).
#define STICK_GAIN_SHIFT 8

// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilise better when full throttle applied
#define MAX_COLLECTIVE 1000      // 95
/*** END DEFINES ***/

/*** BEGIN TYPES ***/
struct RX_STATE_S{
	int16_t roll;
	int16_t pitch;
	int16_t collective;
	int16_t yaw;
#ifdef TWIN_COPTER
	int16_t orgPitch;
#endif
};
/*** END TYPES ***/

/*** BEGIN PROTOTYPES ***/
void receiverSetup(void);
void receiverGetChannels(struct RX_STATE_S *state);
void receiverStickCenterManual(void);
void receiverStickCenterAutomatic(void);
/*** END PROTOTYPES ***/

#endif
