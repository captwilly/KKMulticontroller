#include "gyros.h"

#include "receiver.h"
#include "settings.h"

/*** BEGIN PROTOTYPES ***/
static void init_adc(void);
static uint16_t read_adc(uint8_t channel);
/*** END PROTOTYPES ***/

/*** BEGIN VARIABLES ***/
static struct GYRO_STATE_S gyroZeroPoint;
/*** END VARIABLES ***/

void init_adc(void)
{
  DIDR0  = 0b00111111;  // Digital Input Disable Register - ADC5..0 Digital Input Disable
  ADCSRB  = 0b00000000;  // ADC Control and Status Register B - ADTS2:0
}

void gyrosSetup(void)
{
  GYRO_YAW_DIR    = INPUT;
  GYRO_PITCH_DIR  = INPUT;
  GYRO_ROLL_DIR   = INPUT;
  GAIN_YAW_DIR    = INPUT;
  GAIN_PITCH_DIR  = INPUT;
  GAIN_ROLL_DIR   = INPUT;
  
  init_adc();
}


uint16_t read_adc(uint8_t channel)
{
  ADMUX  = channel;            // set channel
  ADCSRA  = _BV(ADEN) | _BV(ADSC) | _BV(ADPS1) | _BV(ADPS2);  // 0b11000110

  while(ADCSRA & _BV(ADSC))
    ;  // wait to complete
  return ADCW;
}

/*
 * ADC reads 10-bit results (0-1023), so we cannot just multiply Gyro ADC
 * by Gain ADC, or we can wrap results. Full ADC range in a 16 bit value
 * is ADC shifted left by 6, so we scale the gain to 6-bit by shifting
 * right by 10 - 6 = 4 bits.
 */
void gyrosReadGainPots(struct GYRO_GAIN_ADC_S *pots)
{
  // read roll gain
  pots->roll = GAIN_POT_REVERSE read_adc(GAIN_ROLL_ADC_CH);

  // read pitch gain
  pots->pitch = GAIN_POT_REVERSE read_adc(GAIN_PITCH_ADC_CH);

  // read yaw gain
  pots->yaw = GAIN_POT_REVERSE read_adc(GAIN_YAW_ADC_CH);
}

void gyrosRead(struct GYRO_STATE_S *state)
{
  // read roll gyro
  state->roll = read_adc(GYRO_ROLL_ADC_CH) - gyroZeroPoint.roll;
  if(Config.RollGyroDirection == GYRO_NORMAL)
	  state->roll = -state->roll;

  // read pitch gyro
  state->pitch = read_adc(GYRO_PITCH_ADC_CH) - gyroZeroPoint.pitch;
  if(Config.PitchGyroDirection == GYRO_NORMAL)
	  state->pitch = -state->pitch;

#ifdef EXTERNAL_YAW_GYRO
  state->yaw = 0;
#else
  // read yaw gyro
  state->yaw = read_adc(GYRO_YAW_ADC_CH) - gyroZeroPoint.yaw;
  if(Config.YawGyroDirection == GYRO_NORMAL)
	  state->yaw = -state->yaw;
#endif
}

void gyrosCalibrate(void)
{
  struct GYRO_STATE_S gyro;
  uint8_t i;

  // get/set gyro zero value (average of 16 readings)
  gyroZeroPoint.roll = 0;
  gyroZeroPoint.pitch = 0;
  gyroZeroPoint.yaw = 0;

  for(i = 0;i < 16;i++) {
    gyrosRead(&gyro);

    gyroZeroPoint.roll += gyro.roll;
    gyroZeroPoint.pitch += gyro.pitch;
    gyroZeroPoint.yaw += gyro.yaw;
  }

  gyroZeroPoint.roll = (gyroZeroPoint.roll + 8) >> 4;
  gyroZeroPoint.pitch = (gyroZeroPoint.pitch + 8) >> 4;
  gyroZeroPoint.yaw = (gyroZeroPoint.yaw + 8) >> 4;
}

void gyrosReverse(void)
{
  struct RX_STATE_S rxState;

  // flash LED 3 times
  for(uint8_t i = 0;i < 3;i++) {
    LED = 1;
    _delay_ms(25);
    LED = 0;
    _delay_ms(25);
  }

  while(1) {
    receiverGetChannels(&rxState);

    if(rxState.roll < -STICK_THROW) {  // normal(left)
      Config.RollGyroDirection = GYRO_NORMAL;
      Save_Config_to_EEPROM();
      LED = 1;
    } if(rxState.roll > STICK_THROW) {  // reverse(right)
      Config.RollGyroDirection = GYRO_REVERSED;
      Save_Config_to_EEPROM();
      LED = 1;
    } else if(rxState.pitch < -STICK_THROW) { // normal(up)
      Config.PitchGyroDirection = GYRO_NORMAL;
      Save_Config_to_EEPROM();
      LED = 1;
    } else if(rxState.pitch > STICK_THROW) { // reverse(down)
      Config.PitchGyroDirection = GYRO_REVERSED;
      Save_Config_to_EEPROM();
      LED = 1;
    } else if(rxState.yaw < -STICK_THROW) { // normal(left)
      Config.YawGyroDirection = GYRO_NORMAL;
      Save_Config_to_EEPROM();
      LED = 1;
    } else if(rxState.yaw > STICK_THROW) { // reverse(right)
      Config.YawGyroDirection = GYRO_REVERSED;
      Save_Config_to_EEPROM();
      LED = 1;
    }

    _delay_ms(50);
    LED = 0;

  }
}
