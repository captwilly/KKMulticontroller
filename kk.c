#include "common.h"

#include "gyros.h"
#include "settings.h"
#include "receiver.h"
#include "motors.h"
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>


FUSES = {
        .low = FUSE_SUT0 & FUSE_CKSEL3 & FUSE_CKSEL2 & FUSE_CKSEL0, // 0xE2
        .high = FUSE_SPIEN & FUSE_EESAVE & FUSE_BOOTRST,            // 0xd6
        .extended = FUSE_BODLEVEL2 & FUSE_BODLEVEL1,                // 0x04
}; // beware that these are not programmed automatically


static void setup(void);
static void main_loop(void);
int main(void);


static void setup() {
    struct GYRO_GAIN_ADC_S pots;
    MCUCR = _BV(PUD); // Disable hardware pull-up

    LED_DIR = OUTPUT;
    LED = 0;

    receiverSetup();
    gyrosSetup();
    motorsSetup();

    /*
     * This suits my ATmega88A: no Tx trim, output timings perfect.
     * See doc8271.pdf page ~401; beware the step at 128. -Simon
     */
    if (OSCCAL == 0x9d)
        OSCCAL = 0x9f;

    /*
     * timer2 8bit - run at 8MHz / 1024 = 7812.5KHz, just used for arming
     */
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);

    /*
     * Flash the LED once at power on
     */
    LED = 1;
    _delay_ms(150);
    LED = 0;

    sei();

    _delay_ms(1500);

    gyrosReadGainPots(&pots);
    // 5% threshold
    bool pitchMin = (pots.pitch < (ADC_MAX * 5) / 100);
    bool rollMin = (pots.roll < (ADC_MAX * 5) / 100);
    bool yawMin = (pots.yaw < (ADC_MAX * 5) / 100);

    // Clear config
    if (pitchMin && rollMin && yawMin) {
        settingsClearAll();
    }
    // Motor identification
    else if (pitchMin && yawMin) {
        motorsIdentify();
    }
    // Automatic stick centering
    else if (pitchMin && rollMin) {
        receiverStickCenterAutomatic();
    }
    // Future use
    //else if(rollMin && yawMin)      { }
    // Stick Centering Test
    else if (pitchMin) {
        receiverStickCenterManual();
    }
    // Gyro direction reversing
    else if (rollMin) {
        gyrosReverse();
    }
    // ESC throttle calibration
    else if (yawMin) {
        motorsThrottleCalibration();
    }
}

void setMotorZero(
#if defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
        bool returnToInitState
#else
        void
#endif
    ){
    struct MT_STATE_S motors;

    memset(&motors, 0, sizeof(struct MT_STATE_S));

#ifdef SINGLE_COPTER
            motors.m2out = 840;
            motors.m3out = 840;
            motors.m4out = 840;
            motors.m5out = 840;
#elif defined(DUAL_COPTER)
            if(returnToInitState) {
                motors.m3out = 500;
                motors.m4out = 500;
            }
#elif defined(TWIN_COPTER)
            if(returnToInitState) {
                motors.m3out = 500;
                motors.m4out = 500;
                motors.m5out = 500;
                motors.m6out = 500;
            }
#elif defined(TRI_COPTER)
            if(returnToInitState)
            motors.m4out = 500;
#endif
            motorOutputPPM(&motors);
}

static inline void main_loop() {
    static bool Armed = false;
    static uint16_t Change_Arming = 0;
    static uint8_t Arming_TCNT2 = 0;
    int16_t error, emax = 1023;
    int16_t imax, derivative;
    struct MT_STATE_S motors;
    struct RX_STATE_S rxState;
    struct GYRO_GAIN_ADC_S pots;
    struct GYRO_STATE_S gyro;
    struct GYRO_STATE_S integral;   // PID integral term
    struct GYRO_STATE_S last_error; // Last proportional error

    while (true) {

        memset(&motors, 0, sizeof(struct MT_STATE_S));
        motorOutputPPM(&motors);

    	FOREVER {
//    		memset(&motors, 0, sizeof(struct MT_STATE_S));
    	    motors.m1out = 200;
    	    motors.m2out = 200;
    	    motors.m3out = 200;
    	    motors.m4out = 200;
    		motorOutputPPM(&motors);
    	}

        LED = Armed;

        receiverGetChannels(&rxState);

        if (rxState.collective <= 50) {
            // Check for stick arming (Timer2 at 8MHz/1024 = 7812.5KHz)
            Change_Arming += (uint8_t) (TCNT2 - Arming_TCNT2);
            Arming_TCNT2 = TCNT2;

            if (Armed) {
                if (rxState.yaw < STICK_THROW ||
                        abs(rxState.pitch) > STICK_THROW) {
                    Change_Arming = 0; // re-set count
                }
            } else {
                if (rxState.yaw > -STICK_THROW ||
                        abs(rxState.pitch) > STICK_THROW) {
                    Change_Arming = 0; // re-set count
                }
            }

            // 3Sec / 0.000128 = 23437 = 0x5B8D or
            // 2.5Sec / 0.000128 = 19531 = 0x4C4B
            // 0.5Sec / 0.000128 = 3906 = 0x0F42
            // TODO: get rid of magic numbers
            if (Change_Arming > 0x0F42) {
                Armed = !Armed;
                if (Armed)
                    gyrosCalibrate();
            }

            /* turn off motors */
#if defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
            setMotorZero(!Armed);
#else
            setMotorZero();
#endif
        }
        else if(!Armed){
            /* turn off motors */
#if defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
            setMotorZero(true);
#else
            setMotorZero();
#endif
        }
        else {
            gyrosRead(&gyro);
            gyrosReadGainPots(&pots);

            // Start mixing

            imax = MAX(rxState.collective, 0);
            imax >>= 3; /* 1000 -> 200 */

            /* Calculate roll output - Test without props!! */
            rxState.roll = ((int32_t) rxState.roll * (uint32_t) pots.roll)
                    >> STICK_GAIN_SHIFT;
            gyro.roll = ((int32_t) gyro.roll * (uint32_t) pots.roll)
                    >> GYRO_GAIN_SHIFT;
            rxState.roll -= gyro.roll;


            /* Calculate pitch output - Test without props!! */
            rxState.pitch = ((int32_t) rxState.pitch * (uint32_t) pots.pitch)
                    >> STICK_GAIN_SHIFT;
            gyro.pitch = ((int32_t) gyro.pitch * (uint32_t) pots.pitch)
                    >> GYRO_GAIN_SHIFT;
            rxState.pitch -= gyro.pitch;


            /* Calculate yaw output - Test without props!! */
            rxState.yaw = ((int32_t) rxState.yaw * (uint32_t) pots.yaw)
                    >> STICK_GAIN_SHIFT;
            gyro.yaw = ((int32_t) gyro.yaw * (uint32_t) pots.yaw)
                    >> GYRO_GAIN_SHIFT;

            error = rxState.yaw - gyro.yaw;
            if (error > emax)
                error = emax;
            else if (error < -emax)
                error = -emax;
            integral.yaw += error;
            if (integral.yaw > imax)
                integral.yaw = imax;
            else if (integral.yaw < -imax)
                integral.yaw = -imax;
            derivative = error - last_error.yaw;
            last_error.yaw = error;
            rxState.yaw += error + (integral.yaw >> 4) + (derivative >> 4);


            // Apply calculation results to motors output
#ifdef SINGLE_COPTER
            rxState.collective = MIN(rxState.collective, MAX_COLLECTIVE);

            motors.m1out = rxState.collective;
            motors.m2out = 840; // 840
            motors.m3out = 840; // 840
            motors.m4out = 945; // 840 + 840/8
            motors.m5out = 945; // 840 + 840/8

            motors.m2out += rxState.roll;
            motors.m4out -= rxState.roll;

            motors.m3out += rxState.pitch;
            motors.m5out -= rxState.pitch;

            motors.m2out += rxState.yaw;
            motors.m3out += rxState.yaw;
            motors.m4out += rxState.yaw;
            motors.m5out += rxState.yaw;
#elif defined(DUAL_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = 500;
            motors.m4out = 500;

            motors.m4out += rxState.roll;

            motors.m3out += rxState.pitch;

            motors.m1out -= rxState.yaw;
            motors.m2out += rxState.yaw;
#elif defined(TWIN_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = 500;
            motors.m4out = 500;
            motors.m5out = 500; // Optional
            motors.m6out = 500; // Optional

            rxState.roll = (rxState.roll * 7) >> 3; // Approximation of sin(60) without div
            motors.m1out += rxState.roll;
            motors.m2out -= rxState.roll;

            motors.m3out -= SERVO_REVERSE rxState.pitch;
            motors.m4out += SERVO_REVERSE rxState.pitch;
            // Stick Only, Optional
            rxState.orgPitch = abs(rxState.orgPitch);
            motors.m5out += rxState.orgPitch; // Tain Servo-Optional, Down Only
            motors.m6out -= rxState.orgPitch; // Tain Servo-Optional, Down Only (Reverse)

            motors.m3out += SERVO_REVERSE(rxState.yaw >> 1);
            motors.m4out += SERVO_REVERSE(rxState.yaw >> 1);
#elif defined(TRI_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = rxState.collective;
            motors.m4out = 500;

            rxState.roll = (rxState.roll * 7) >> 3; // (.875 versus .86602540)
            motors.m1out += rxState.roll;
            motors.m2out -= rxState.roll;

            motors.m3out -= rxState.pitch;
            rxState.pitch = (rxState.pitch >> 1); // cosine of 60
            motors.m1out += rxState.pitch;
            motors.m2out += rxState.pitch;

            motors.m4out += SERVO_REVERSE rxState.yaw;
#elif defined(QUAD_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = rxState.collective;
            motors.m4out = rxState.collective;

            motors.m2out += rxState.roll;
            motors.m3out -= rxState.roll;

            motors.m1out += rxState.pitch;
            motors.m4out -= rxState.pitch;

            motors.m1out -= rxState.yaw;
            motors.m2out += rxState.yaw;
            motors.m3out += rxState.yaw;
            motors.m4out -= rxState.yaw;
#elif defined(QUAD_X_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = rxState.collective;
            motors.m4out = rxState.collective;

            rxState.roll = rxState.roll >> 1;
            motors.m1out += rxState.roll;
            motors.m2out -= rxState.roll;
            motors.m3out -= rxState.roll;
            motors.m4out += rxState.roll;

            rxState.pitch = (rxState.pitch >> 1); // cosine of 60
            motors.m1out += rxState.pitch;
            motors.m2out += rxState.pitch;
            motors.m3out -= rxState.pitch;
            motors.m4out -= rxState.pitch;

            motors.m1out -= rxState.yaw;
            motors.m2out += rxState.yaw;
            motors.m3out -= rxState.yaw;
            motors.m4out += rxState.yaw;
#elif defined(Y4_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = rxState.collective * 3 / 4; // 25% Down
            motors.m4out = rxState.collective * 3 / 4; // 25% Down

            rxState.roll = (rxState.roll * 7) >> 3;
            motors.m1out += rxState.roll;
            motors.m2out -= rxState.roll;

            motors.m1out += rxState.pitch;
            motors.m2out += rxState.pitch;
            motors.m3out -= rxState.pitch;
            motors.m4out -= rxState.pitch;

            if((motors.m3out - rxState.yaw) < 100)
            rxState.yaw = motors.m3out - 100; // Yaw Range Limit
            if((motors.m3out - rxState.yaw) > 1000)
            rxState.yaw = motors.m3out - 1000; // Yaw Range Limit

            if((motors.m4out + rxState.yaw) < 100)
            rxState.yaw = 100 - motors.m4out; // Yaw Range Limit
            if((motors.m4out + rxState.yaw) > 1000)
            rxState.yaw = 1000 - motors.m4out; // Yaw Range Limit

            motors.m3out -= rxState.yaw;
            motors.m4out += rxState.yaw;
#elif defined(HEX_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = rxState.collective;
            motors.m4out = rxState.collective;
            motors.m5out = rxState.collective;
            motors.m6out = rxState.collective;

            rxState.roll = (rxState.roll * 7) >> 3;
            motors.m2out -= rxState.roll;
            motors.m3out -= rxState.roll;
            motors.m5out += rxState.roll;
            motors.m6out += rxState.roll;

            motors.m1out += rxState.pitch;
            motors.m4out -= rxState.pitch;
            rxState.pitch = (rxState.pitch >> 2);
            motors.m2out += rxState.pitch;
            motors.m3out -= rxState.pitch;
            motors.m5out -= rxState.pitch;
            motors.m6out += rxState.pitch;

            motors.m1out -= rxState.yaw;
            motors.m2out += rxState.yaw;
            motors.m3out -= rxState.yaw;
            motors.m4out += rxState.yaw;
            motors.m5out -= rxState.yaw;
            motors.m6out += rxState.yaw;
#elif defined(Y6_COPTER)
            motors.m1out = rxState.collective;
            motors.m2out = rxState.collective;
            motors.m3out = rxState.collective;
            motors.m4out = rxState.collective;
            motors.m5out = rxState.collective;
            motors.m6out = rxState.collective;

            rxState.roll = (rxState.roll * 7) >> 3;
            motors.m1out += rxState.roll;
            motors.m2out += rxState.roll;
            motors.m3out -= rxState.roll;
            motors.m4out -= rxState.roll;

            motors.m5out -= rxState.pitch;
            motors.m6out -= rxState.pitch;
            rxState.pitch = (rxState.pitch >> 1); // cosine of 60
            motors.m1out += rxState.pitch;
            motors.m2out += rxState.pitch;
            motors.m3out += rxState.pitch;
            motors.m4out += rxState.pitch;

            motors.m1out -= rxState.yaw;
            motors.m4out -= rxState.yaw;
            motors.m5out -= rxState.yaw;
            motors.m2out += rxState.yaw;
            motors.m3out += rxState.yaw;
            motors.m6out += rxState.yaw;
#endif


#if defined(TRI_COPTER)
            /*
             * Rather than clipping the motor outputs and causing instability
             * at throttle saturation, we pull down the throttle of the other
             * motors. This gives priority to stabilization without a fixed
             * collective limit.
             */
            imax = MAX(motors.m1out, motors.m2out);
            imax = MAX(motors.m3out, imax);

            imax -= 1000;
            if(imax > 0) {
                motors.m1out -= imax;
                motors.m2out -= imax;
                motors.m3out -= imax;
            }
#endif

            /* Limit the lowest value to avoid stopping of motor if motor value
             *  is under-saturated */
            motors.m1out = MAX(MOTOR_LOWEST_VALUE, motors.m1out);
            motors.m2out = MAX(MOTOR_LOWEST_VALUE, motors.m2out);

#if defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || \
    defined(Y4_COPTER)
            motors.m3out = MAX(MOTOR_LOWEST_VALUE, motors.m3out);
#endif
#if defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
            motors.m4out = MAX(MOTOR_LOWEST_VALUE, motors.m4out);
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
            motors.m5out = MAX(MOTOR_LOWEST_VALUE, motors.m5out);
            motors.m6out = MAX(MOTOR_LOWEST_VALUE, motors.m6out);
#endif

            LED = 0;
            motorOutputPPM(&motors);
        }
    }
}

int main() {
    setup();
    main_loop();
    return 1;
}
