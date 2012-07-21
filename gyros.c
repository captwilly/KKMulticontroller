#include "gyros.h"

#include "receiver.h"
#include "settings.h"

uint16_t GainInADC[3];        // ADC result
int16_t  gyroADC[3];          // Holds Gyro ADC's
int16_t  gyroZero[3];         // used for calibrating Gyros on ground

void init_adc()
{
  DIDR0  = 0b00111111;  // Digital Input Disable Register - ADC5..0 Digital Input Disable
  ADCSRB  = 0b00000000;  // ADC Control and Status Register B - ADTS2:0
}

void gyrosSetup()
{
  GYRO_YAW_DIR    = INPUT;
  GYRO_PITCH_DIR  = INPUT;
  GYRO_ROLL_DIR   = INPUT;
  GAIN_YAW_DIR    = INPUT;
  GAIN_PITCH_DIR  = INPUT;
  GAIN_ROLL_DIR   = INPUT;
  
  init_adc();
}


void read_adc(uint8_t channel)
{
  ADMUX  = channel;            // set channel
  ADCSRA  = _BV(ADEN) | _BV(ADSC) | _BV(ADPS1) | _BV(ADPS2);  // 0b11000110

  while(ADCSRA & _BV(ADSC))
    ;  // wait to complete
}

/*
 * ADC reads 10-bit results (0-1023), so we cannot just multiply Gyro ADC
 * by Gain ADC, or we can wrap results. Full ADC range in a 16 bit value
 * is ADC shifted left by 6, so we scale the gain to 6-bit by shifting
 * right by 10 - 6 = 4 bits.
 */
void ReadGainPots()
{
  read_adc(3);      // read roll gain ADC3
  GainInADC[ROLL] = GAIN_POT_REVERSE ADCW;

  read_adc(4);      // read pitch gain ADC4
  GainInADC[PITCH] = GAIN_POT_REVERSE ADCW;

  read_adc(5);      // read yaw gain ADC5
  GainInADC[YAW] = GAIN_POT_REVERSE ADCW;
}

void ReadGyros()
{
  read_adc(2);      // read roll gyro ADC2
  gyroADC[ROLL] = ADCW;

  read_adc(1);      // read pitch gyro ADC1
  gyroADC[PITCH] = ADCW;

#ifdef EXTERNAL_YAW_GYRO
  gyroADC[YAW] = 0;
#else
  read_adc(0);      // read yaw gyro ADC0
  gyroADC[YAW] = ADCW;
#endif
}

void CalibrateGyros()
{
  uint8_t i;

  ReadGainPots();  // about time we did this !

  // get/set gyro zero value (average of 16 readings)
  gyroZero[ROLL] = 0;
  gyroZero[PITCH] = 0;
  gyroZero[YAW] = 0;

  for(i = 0;i < 16;i++) {
    ReadGyros();

    gyroZero[ROLL]+= gyroADC[ROLL];
    gyroZero[PITCH]+= gyroADC[PITCH];
    gyroZero[YAW]+= gyroADC[YAW];
  }

  gyroZero[ROLL] = (gyroZero[ROLL] + 8) >> 4;
  gyroZero[PITCH] = (gyroZero[PITCH] + 8) >> 4;
  gyroZero[YAW] = (gyroZero[YAW] + 8) >> 4;
}

void gyrosReverse()
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
