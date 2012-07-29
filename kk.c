#include "config.h"
#include "gyros.h"
#include "settings.h"
#include "receiver.h"
#include "motors.h"


FUSES = {
        .low = FUSE_SUT0 & FUSE_CKSEL3 & FUSE_CKSEL2 & FUSE_CKSEL0, // 0xE2
        .high = FUSE_SPIEN & FUSE_EESAVE & FUSE_BOOTRST,            // 0xd6
        .extended = FUSE_BODLEVEL2 & FUSE_BODLEVEL1,                // 0x04
}; // beware that these are not programmed automatically


bool Armed;


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

    Armed = false;

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

static inline void main_loop() {
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

        receiverGetChannels(&rxState);

        if (rxState.collective <= 0) {
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
                        abs(rxState.pitch) > STICK_THROW){
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
        }

        gyrosRead(&gyro);

        LED = Armed;

        //--- Start mixing by setting collective to motor outputs

        rxState.collective = (rxState.collective * 10) >> 3; // 0-800 -> 0-1000

#ifndef SINGLE_COPTER
        if (rxState.collective > MAX_COLLECTIVE)
            rxState.collective = MAX_COLLECTIVE;
#endif

#ifdef SINGLE_COPTER
        motors.m1out = rxState.collective;
        motors.m2out = 840; // 840
        motors.m3out = 840; // 840
        motors.m4out = 945; // 840 + 840/8
        motors.m5out = 945; // 840 + 840/8
#elif defined(DUAL_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = 500;
        motors.m4out = 500;
#elif defined(TWIN_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = 500;
        motors.m4out = 500;
        motors.m5out = 500; // Optional
        motors.m6out = 500; // Optional
#elif defined(TRI_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = rxState.collective;
        motors.m4out = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = rxState.collective;
        motors.m4out = rxState.collective;
#elif defined(Y4_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = rxState.collective * 3 / 4; // 25% Down
        motors.m4out = rxState.collective * 3 / 4; // 25% Down
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = rxState.collective;
        motors.m4out = rxState.collective;
        motors.m5out = rxState.collective;
        motors.m6out = rxState.collective;
#endif

        imax = rxState.collective;
        if (imax < 0)
            imax = 0;
        imax >>= 3; /* 1000 -> 200 */

        /* Calculate roll output - Test without props!! */

        gyrosReadGainPots(&pots);

        rxState.roll = ((int32_t) rxState.roll * (uint32_t) pots.roll)
                >> STICK_GAIN_SHIFT;
        gyro.roll = ((int32_t) gyro.roll * (uint32_t) pots.roll)
                >> GYRO_GAIN_SHIFT;

        if (Armed) {
            if (0) {
                error = rxState.roll - gyro.roll;
                if (error > emax)
                    error = emax;
                else if (error < -emax)
                    error = -emax;
                integral.roll += error;
                if (integral.roll > imax)
                    integral.roll = imax;
                else if (integral.roll < -imax)
                    integral.roll = -imax;
                derivative = error - last_error.roll;
                last_error.roll = error;
                rxState.roll += error + (integral.roll >> 2)
                        + (derivative >> 2);
            } else {
                rxState.roll -= gyro.roll;
            }
        }

#ifdef SINGLE_COPTER
        motors.m2out += rxState.roll;
        motors.m4out -= rxState.roll;
#elif defined(DUAL_COPTER)
        motors.m4out += rxState.roll;
#elif defined(TWIN_COPTER)
        rxState.roll = (rxState.roll * 7) >> 3; // Approximation of sin(60) without div
        motors.m1out += rxState.roll;
        motors.m2out -= rxState.roll;
#elif defined(TRI_COPTER)
        rxState.roll = (rxState.roll * 7) >> 3; // (.875 versus .86602540)
        motors.m1out += rxState.roll;
        motors.m2out -= rxState.roll;
#elif defined(QUAD_COPTER)
        motors.m2out += rxState.roll;
        motors.m3out -= rxState.roll;
#elif defined(QUAD_X_COPTER)
        rxState.roll = rxState.roll >> 1;
        motors.m1out += rxState.roll;
        motors.m2out -= rxState.roll;
        motors.m3out -= rxState.roll;
        motors.m4out += rxState.roll;
#elif defined(Y4_COPTER)
        rxState.roll = (rxState.roll * 7) >> 3;
        motors.m1out += rxState.roll;
        motors.m2out -= rxState.roll;
#elif defined(HEX_COPTER)
        rxState.roll = (rxState.roll * 7) >> 3;
        motors.m2out -= rxState.roll;
        motors.m3out -= rxState.roll;
        motors.m5out += rxState.roll;
        motors.m6out += rxState.roll;
#elif defined(Y6_COPTER)
        rxState.roll = (rxState.roll * 7) >> 3;
        motors.m1out += rxState.roll;
        motors.m2out += rxState.roll;
        motors.m3out -= rxState.roll;
        motors.m4out -= rxState.roll;
#endif

        /* Calculate pitch output - Test without props!! */

        rxState.pitch = ((int32_t) rxState.pitch * (uint32_t) pots.pitch)
                >> STICK_GAIN_SHIFT;
        gyro.pitch = ((int32_t) gyro.pitch * (uint32_t) pots.pitch)
                >> GYRO_GAIN_SHIFT;

        if (Armed) {
            if (0) {
                error = rxState.pitch - gyro.pitch;
                if (error > emax)
                    error = emax;
                else if (error < -emax)
                    error = -emax;
                integral.pitch += error;
                if (integral.pitch > imax)
                    integral.pitch = imax;
                else if (integral.pitch < -imax)
                    integral.pitch = -imax;
                derivative = error - last_error.pitch;
                last_error.pitch = error;
                rxState.pitch += error + (integral.pitch >> 2) + (derivative
                        >> 2);
            } else {
                rxState.pitch -= gyro.pitch;
            }
        }

#ifdef SINGLE_COPTER
        motors.m3out += rxState.pitch;
        motors.m5out -= rxState.pitch;
#elif defined(DUAL_COPTER)
        motors.m3out += rxState.pitch;
#elif defined(TWIN_COPTER)
        motors.m3out -= SERVO_REVERSE rxState.pitch;
        motors.m4out += SERVO_REVERSE rxState.pitch;
        // Stick Only, Optional
        rxState.orgPitch = abs(rxState.orgPitch);
        motors.m5out += rxState.orgPitch; // Tain Servo-Optional, Down Only
        motors.m6out -= rxState.orgPitch; // Tain Servo-Optional, Down Only (Reverse)
#elif defined(TRI_COPTER)
        motors.m3out -= rxState.pitch;
        rxState.pitch = (rxState.pitch >> 1); // cosine of 60
        motors.m1out += rxState.pitch;
        motors.m2out += rxState.pitch;
#elif defined(QUAD_COPTER)
        motors.m1out += rxState.pitch;
        motors.m4out -= rxState.pitch;
#elif defined(QUAD_X_COPTER)
        rxState.pitch = (rxState.pitch >> 1); // cosine of 60
        motors.m1out += rxState.pitch;
        motors.m2out += rxState.pitch;
        motors.m3out -= rxState.pitch;
        motors.m4out -= rxState.pitch;
#elif defined(Y4_COPTER)
        motors.m1out += rxState.pitch;
        motors.m2out += rxState.pitch;
        motors.m3out -= rxState.pitch;
        motors.m4out -= rxState.pitch;
#elif defined(HEX_COPTER)
        motors.m1out += rxState.pitch;
        motors.m4out -= rxState.pitch;
        rxState.pitch = (rxState.pitch >> 2);
        motors.m2out += rxState.pitch;
        motors.m3out -= rxState.pitch;
        motors.m5out -= rxState.pitch;
        motors.m6out += rxState.pitch;
#elif defined(Y6_COPTER)
        motors.m5out -= rxState.pitch;
        motors.m6out -= rxState.pitch;
        rxState.pitch = (rxState.pitch >> 1); // cosine of 60
        motors.m1out += rxState.pitch;
        motors.m2out += rxState.pitch;
        motors.m3out += rxState.pitch;
        motors.m4out += rxState.pitch;
#endif

        /* Calculate yaw output - Test without props!! */

        rxState.yaw = ((int32_t) rxState.yaw * (uint32_t) pots.yaw)
                >> STICK_GAIN_SHIFT;
        gyro.yaw = ((int32_t) gyro.yaw * (uint32_t) pots.yaw)
                >> GYRO_GAIN_SHIFT;

        if (Armed) {
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
        }

#ifdef SINGLE_COPTER
        motors.m2out += rxState.yaw;
        motors.m3out += rxState.yaw;
        motors.m4out += rxState.yaw;
        motors.m5out += rxState.yaw;
#elif defined(DUAL_COPTER)
        motors.m1out -= rxState.yaw;
        motors.m2out += rxState.yaw;
#elif defined(TWIN_COPTER)
        motors.m3out += SERVO_REVERSE(rxState.yaw >> 1);
        motors.m4out += SERVO_REVERSE(rxState.yaw >> 1);
#elif defined(TRI_COPTER)
        motors.m4out += SERVO_REVERSE rxState.yaw;
#elif defined(QUAD_COPTER)
        motors.m1out -= rxState.yaw;
        motors.m2out += rxState.yaw;
        motors.m3out += rxState.yaw;
        motors.m4out -= rxState.yaw;
#elif defined(QUAD_X_COPTER)
        motors.m1out -= rxState.yaw;
        motors.m2out += rxState.yaw;
        motors.m3out -= rxState.yaw;
        motors.m4out += rxState.yaw;
#elif defined(Y4_COPTER)
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
        motors.m1out -= rxState.yaw;
        motors.m2out += rxState.yaw;
        motors.m3out -= rxState.yaw;
        motors.m4out += rxState.yaw;
        motors.m5out -= rxState.yaw;
        motors.m6out += rxState.yaw;
#elif defined(Y6_COPTER)
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
        imax = motors.m1out;
        if(motors.m2out > imax)
        imax = motors.m2out;
        if(motors.m3out > imax)
        imax = motors.m3out;
        imax -= 1000;
        if(imax > 0) {
            motors.m1out -= imax;
            motors.m2out -= imax;
            motors.m3out -= imax;
        }
#endif

        imax = 114;
        //--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
        if (motors.m1out < imax)
            motors.m1out = imax; // this is the motor idle level
        if (motors.m2out < imax)
            motors.m2out = imax;
#if defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
        if (motors.m3out < imax)
            motors.m3out = imax;
#endif
#if defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
        if (motors.m4out < imax)
            motors.m4out = imax;
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
        if(motors.m5out < imax)
        motors.m5out = imax;
        if(motors.m6out < imax)
        motors.m6out = imax;
#endif

        //--- Output to motor ESC's ---
        if (rxState.collective < 1 || !Armed) {
            /* turn off motors unless armed and collective is non-zero */
#ifdef SINGLE_COPTER
            motors.m1out = 0;
            motors.m2out = 840;
            motors.m3out = 840;
            motors.m4out = 840;
            motors.m5out = 840;
#elif defined(DUAL_COPTER)
            motors.m1out = 0;
            motors.m2out = 0;
            if(!Armed) {
                motors.m3out = 500;
                motors.m4out = 500;
            }
#elif defined(TWIN_COPTER)
            motors.m1out = 0;
            motors.m2out = 0;
            if(!Armed) {
                motors.m3out = 500;
                motors.m4out = 500;
                motors.m5out = 500;
                motors.m6out = 500;
            }
#elif defined(TRI_COPTER)
            motors.m1out = 0;
            motors.m2out = 0;
            motors.m3out = 0;
            if(!Armed)
            motors.m4out = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
            motors.m1out = 0;
            motors.m2out = 0;
            motors.m3out = 0;
            motors.m4out = 0;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
            motors.m1out = 0;
            motors.m2out = 0;
            motors.m3out = 0;
            motors.m4out = 0;
            motors.m5out = 0;
            motors.m6out = 0;
#endif
        }

        LED = 0;
        motorOutputPPM(&motors);
    }
}

int main() {
    setup();
    main_loop();
    return 1;
}
