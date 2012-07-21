#include "config.h"
#include "gyros.h"
#include "settings.h"
#include "receiver.h"
#include "motors.h"

bool Armed;

static int16_t integral[3];          // PID integral term
static int16_t last_error[3];        // Last proportional error

static void setup(void);
static void loop(void);
int main(void);

static void setup()
{
  MCUCR = _BV(PUD);  // Disable hardware pull-up

  receiverSetup();
  gyrosSetup();
  motorsSetup();
  settingsSetup();

  LED_DIR   = OUTPUT;
  LED    = 0;

  /*
   * This suits my ATmega88A: no Tx trim, output timings perfect.
   * See doc8271.pdf page ~401; beware the step at 128. -Simon
   */
  if(OSCCAL == 0x9d)
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

  ReadGainPots();
  ReadGainPots();
  bool pitchMin = (GainInADC[PITCH] < (ADC_MAX * 5) / 100);    // 5% threshold
  bool rollMin =  (GainInADC[ROLL]  < (ADC_MAX * 5) / 100);    // 5% threshold
  bool yawMin =   (GainInADC[YAW]   < (ADC_MAX * 5) / 100);    // 5% threshold

  if(pitchMin && rollMin && yawMin) { settingsClearAll(); }             // Clear config
  else if(pitchMin && yawMin)       { motorsIdentify(); }               // Motor identification
//  else if(pitchMin && rollMin)      { }                                 // Future use
//  else if(rollMin && yawMin)        { }                                 // Future use
  else if(pitchMin)                 { receiverStickCenter(); }          // Stick Centering Test
  else if(rollMin)                  { gyrosReverse(); }                 // Gyro direction reversing
  else if(yawMin)                   { motorsThrottleCalibration(); }    // ESC throttle calibration
}

static inline void loop()
{
  static uint16_t Change_Arming = 0;
  static uint8_t Arming_TCNT2 = 0;
  int16_t error, emax = 1023;
  int16_t imax, derivative;
  struct MT_STATE_S motors;

  RxGetChannels();

  if(RxInCollective <= 0) {
    // Check for stick arming (Timer2 at 8MHz/1024 = 7812.5KHz)
    Change_Arming += (uint8_t)(TCNT2 - Arming_TCNT2);
    Arming_TCNT2 = TCNT2;

    if(Armed) {
      if(RxInYaw < STICK_THROW || abs(RxInPitch) > STICK_THROW)
        Change_Arming = 0;    // re-set count
    } else {
      if(RxInYaw > -STICK_THROW || abs(RxInPitch) > STICK_THROW)
        Change_Arming = 0;    // re-set count
    }

    // 3Sec / 0.000128 = 23437 = 0x5B8D or
    // 2.5Sec / 0.000128 = 19531 = 0x4C4B
    // 0.5Sec / 0.000128 = 3906 = 0x0F42
    if(Change_Arming > 0x0F42) {
      Armed = !Armed;
      if(Armed)
        CalibrateGyros();
    }
  }

  ReadGyros();

  LED = Armed;

  gyroADC[ROLL]-= gyroZero[ROLL];
  gyroADC[PITCH]-= gyroZero[PITCH];
  gyroADC[YAW]-= gyroZero[YAW];

  //--- Start mixing by setting collective to motor outputs

  RxInCollective = (RxInCollective * 10) >> 3;  // 0-800 -> 0-1000

#ifndef SINGLE_COPTER
  if(RxInCollective > MAX_COLLECTIVE)
    RxInCollective = MAX_COLLECTIVE;
#endif

#ifdef SINGLE_COPTER
  motors.m1out = RxInCollective;
  motors.m2out = 840;  // 840
  motors.m3out = 840;  // 840
  motors.m4out = 945;  // 840 + 840/8
  motors.m5out = 945;  // 840 + 840/8
#elif defined(DUAL_COPTER)
  motors.m1out = RxInCollective;
  motors.m2out = RxInCollective;
  motors.m3out = 500;
  motors.m4out = 500;
#elif defined(TWIN_COPTER)
  motors.m1out = RxInCollective;
  motors.m2out = RxInCollective;
  motors.m3out = 500;
  motors.m4out = 500;
  motors.m5out = 500;  // Optional
  motors.m6out = 500;  // Optional
#elif defined(TRI_COPTER)
  motors.m1out = RxInCollective;
  motors.m2out = RxInCollective;
  motors.m3out = RxInCollective;
  motors.m4out = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER)
  motors.m1out = RxInCollective;
  motors.m2out = RxInCollective;
  motors.m3out = RxInCollective;
  motors.m4out = RxInCollective;
#elif defined(Y4_COPTER)
  motors.m1out = RxInCollective;
  motors.m2out = RxInCollective;
  motors.m3out = RxInCollective * 3 / 4;    // 25% Down
  motors.m4out = RxInCollective * 3 / 4;    // 25% Down
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
  motors.m1out = RxInCollective;
  motors.m2out = RxInCollective;
  motors.m3out = RxInCollective;
  motors.m4out = RxInCollective;
  motors.m5out = RxInCollective;
  motors.m6out = RxInCollective;
#endif

  imax = RxInCollective;
  if(imax < 0)
    imax = 0;
  imax>>= 3;  /* 1000 -> 200 */

  /* Calculate roll output - Test without props!! */

  RxInRoll = ((int32_t)RxInRoll * (uint32_t)GainInADC[ROLL]) >> STICK_GAIN_SHIFT;
  gyroADC[ROLL] = ((int32_t)gyroADC[ROLL] * (uint32_t)GainInADC[ROLL]) >> GYRO_GAIN_SHIFT;
  if(Config.RollGyroDirection == GYRO_NORMAL)
    gyroADC[ROLL] = -gyroADC[ROLL];

  if(Armed) {
    if(0) {
      error = RxInRoll - gyroADC[ROLL];
      if(error > emax)
        error = emax;
      else if(error < -emax)
        error = -emax;
      integral[ROLL]+= error;
      if(integral[ROLL] > imax)
        integral[ROLL] = imax;
      else if(integral[ROLL] < -imax)
        integral[ROLL] = -imax;
      derivative = error - last_error[ROLL];
      last_error[ROLL] = error;
      RxInRoll += error + (integral[ROLL] >> 2) + (derivative >> 2);
    } else {
      RxInRoll -= gyroADC[ROLL];
    }
  }

#ifdef SINGLE_COPTER
  motors.m2out += RxInRoll;
  motors.m4out -= RxInRoll;
#elif defined(DUAL_COPTER)
  motors.m4out += RxInRoll;
#elif defined(TWIN_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;  // Approximation of sin(60) without div
  motors.m1out += RxInRoll;
  motors.m2out -= RxInRoll;
#elif defined(TRI_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;  // (.875 versus .86602540)
  motors.m1out += RxInRoll;
  motors.m2out -= RxInRoll;
#elif defined(QUAD_COPTER)
  motors.m2out += RxInRoll;
  motors.m3out -= RxInRoll;
#elif defined(QUAD_X_COPTER)
  RxInRoll = RxInRoll >> 1;
  motors.m1out += RxInRoll;
  motors.m2out -= RxInRoll;
  motors.m3out -= RxInRoll;
  motors.m4out += RxInRoll;
#elif defined(Y4_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  motors.m1out += RxInRoll;
  motors.m2out -= RxInRoll;
#elif defined(HEX_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  motors.m2out -= RxInRoll;
  motors.m3out -= RxInRoll;
  motors.m5out += RxInRoll;
  motors.m6out += RxInRoll;
#elif defined(Y6_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  motors.m1out += RxInRoll;
  motors.m2out += RxInRoll;
  motors.m3out -= RxInRoll;
  motors.m4out -= RxInRoll;
#endif

  /* Calculate pitch output - Test without props!! */

  RxInPitch = ((int32_t)RxInPitch * (uint32_t)GainInADC[PITCH]) >> STICK_GAIN_SHIFT;
  gyroADC[PITCH] = ((int32_t)gyroADC[PITCH] * (uint32_t)GainInADC[PITCH]) >> GYRO_GAIN_SHIFT;
  if(Config.PitchGyroDirection == GYRO_NORMAL)
    gyroADC[PITCH] = -gyroADC[PITCH];

  if(Armed) {
    if(0) {
      error = RxInPitch - gyroADC[PITCH];
      if(error > emax)
        error = emax;
      else if(error < -emax)
        error = -emax;
      integral[PITCH]+= error;
      if(integral[PITCH] > imax)
        integral[PITCH] = imax;
      else if(integral[PITCH] < -imax)
        integral[PITCH] = -imax;
      derivative = error - last_error[PITCH];
      last_error[PITCH] = error;
      RxInPitch += error + (integral[PITCH] >> 2) + (derivative >> 2);
    } else {
      RxInPitch -= gyroADC[PITCH];
    }
  }

#ifdef SINGLE_COPTER
  motors.m3out += RxInPitch;
  motors.m5out -= RxInPitch;
#elif defined(DUAL_COPTER)
  motors.m3out += RxInPitch;
#elif defined(TWIN_COPTER)
  motors.m3out -= SERVO_REVERSE RxInPitch;
  motors.m4out += SERVO_REVERSE RxInPitch;
  // Stick Only, Optional
  RxInOrgPitch = abs(RxInOrgPitch);
  motors.m5out += RxInOrgPitch;      // Tain Servo-Optional, Down Only
  motors.m6out -= RxInOrgPitch;      // Tain Servo-Optional, Down Only (Reverse)
#elif defined(TRI_COPTER)
  motors.m3out -= RxInPitch;
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  motors.m1out += RxInPitch;
  motors.m2out += RxInPitch;
#elif defined(QUAD_COPTER)
  motors.m1out += RxInPitch;
  motors.m4out -= RxInPitch;
#elif defined(QUAD_X_COPTER)
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  motors.m1out += RxInPitch;
  motors.m2out += RxInPitch;
  motors.m3out -= RxInPitch;
  motors.m4out -= RxInPitch;
#elif defined(Y4_COPTER)
  motors.m1out += RxInPitch;
  motors.m2out += RxInPitch;
  motors.m3out -= RxInPitch;
  motors.m4out -= RxInPitch;
#elif defined(HEX_COPTER)
  motors.m1out += RxInPitch;
  motors.m4out -= RxInPitch;
  RxInPitch = (RxInPitch >> 2);
  motors.m2out += RxInPitch;
  motors.m3out -= RxInPitch;
  motors.m5out -= RxInPitch;
  motors.m6out += RxInPitch;
#elif defined(Y6_COPTER)
  motors.m5out -= RxInPitch;
  motors.m6out -= RxInPitch;
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  motors.m1out += RxInPitch;
  motors.m2out += RxInPitch;
  motors.m3out += RxInPitch;
  motors.m4out += RxInPitch;
#endif

  /* Calculate yaw output - Test without props!! */

  RxInYaw = ((int32_t)RxInYaw * (uint32_t)GainInADC[YAW]) >> STICK_GAIN_SHIFT;
  gyroADC[YAW] = ((int32_t)gyroADC[YAW] * (uint32_t)GainInADC[YAW]) >> GYRO_GAIN_SHIFT;
  if(Config.YawGyroDirection == GYRO_NORMAL)
    gyroADC[YAW] = -gyroADC[YAW];

  if(Armed) {
    error = RxInYaw - gyroADC[YAW];
    if(error > emax)
      error = emax;
    else if(error < -emax)
      error = -emax;
    integral[YAW]+= error;
    if(integral[YAW] > imax)
      integral[YAW] = imax;
    else if(integral[YAW] < -imax)
      integral[YAW] = -imax;
    derivative = error - last_error[YAW];
    last_error[YAW] = error;
    RxInYaw += error + (integral[YAW] >> 4) + (derivative >> 4);
  }

#ifdef SINGLE_COPTER
  motors.m2out += RxInYaw;
  motors.m3out += RxInYaw;
  motors.m4out += RxInYaw;
  motors.m5out += RxInYaw;
#elif defined(DUAL_COPTER)
  motors.m1out -= RxInYaw;
  motors.m2out += RxInYaw;
#elif defined(TWIN_COPTER)
  motors.m3out += SERVO_REVERSE(RxInYaw >> 1);
  motors.m4out += SERVO_REVERSE(RxInYaw >> 1);
#elif defined(TRI_COPTER)
  motors.m4out += SERVO_REVERSE RxInYaw;
#elif defined(QUAD_COPTER)
  motors.m1out -= RxInYaw;
  motors.m2out += RxInYaw;
  motors.m3out += RxInYaw;
  motors.m4out -= RxInYaw;
#elif defined(QUAD_X_COPTER)
  motors.m1out -= RxInYaw;
  motors.m2out += RxInYaw;
  motors.m3out -= RxInYaw;
  motors.m4out += RxInYaw;
#elif defined(Y4_COPTER)
  if((motors.m3out - RxInYaw) < 100)
    RxInYaw = motors.m3out - 100;  // Yaw Range Limit
  if((motors.m3out - RxInYaw) > 1000)
    RxInYaw = motors.m3out - 1000;  // Yaw Range Limit

  if((motors.m4out + RxInYaw) < 100)
    RxInYaw = 100 - motors.m4out;  // Yaw Range Limit
  if((motors.m4out + RxInYaw) > 1000)
    RxInYaw = 1000 - motors.m4out;  // Yaw Range Limit

  motors.m3out -= RxInYaw;
  motors.m4out += RxInYaw;
#elif defined(HEX_COPTER)
  motors.m1out -= RxInYaw;
  motors.m2out += RxInYaw;
  motors.m3out -= RxInYaw;
  motors.m4out += RxInYaw;
  motors.m5out -= RxInYaw;
  motors.m6out += RxInYaw;
#elif defined(Y6_COPTER)
  motors.m1out -= RxInYaw;
  motors.m4out -= RxInYaw;
  motors.m5out -= RxInYaw;
  motors.m2out += RxInYaw;
  motors.m3out += RxInYaw;
  motors.m6out += RxInYaw;
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
  if(motors.m1out < imax)
    motors.m1out = imax;  // this is the motor idle level
  if(motors.m2out < imax)
    motors.m2out = imax;
#if defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
  if(motors.m3out < imax)
    motors.m3out = imax;
#endif
#if defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
  if(motors.m4out < imax)
    motors.m4out = imax;
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
  if(motors.m5out < imax)
    motors.m5out = imax;
  if(motors.m6out < imax)
    motors.m6out = imax;
#endif

  //--- Output to motor ESC's ---
  if(RxInCollective < 1 || !Armed) {
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

int main()
{
  setup();

  while(1)
    loop();
  return 1;
}
