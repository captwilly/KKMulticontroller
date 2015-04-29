/*********************************************************************
 *
 *********************************************************************
 * FileName:	    io_cfg.h
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

/** I N C L U D E S *************************************************/
#include "typedefs.h"

#define RX_ROLL             REGISTER_BIT(PIND,1)
#define RX_PITCH            REGISTER_BIT(PIND,2)  // INT0
#define RX_COLL             REGISTER_BIT(PIND,3)  // INT1
#define RX_YAW              REGISTER_BIT(PINB,7)
#define RX_ROLL_DIR         REGISTER_BIT(DDRD,1)
#define RX_PITCH_DIR        REGISTER_BIT(DDRD,2)
#define RX_COLL_DIR         REGISTER_BIT(DDRD,3)
#define RX_YAW_DIR          REGISTER_BIT(DDRB,7)

#if defined(RX_CONFIG) && !defined(ATTITUDE_SENSOR)
#define RX_CONFIG_1			REGISTER_BIT(PIND,6)
#define RX_CONFIG_2			REGISTER_BIT(PIND,5)
#define RX_CONFIG_1_DIR		REGISTER_BIT((DDRD,6)
#define RX_CONFIG_2_DIR		REGISTER_BIT((DDRD,5)
#endif

#define GYRO_ROLL           REGISTER_BIT(PINC,2)
#define GYRO_PITCH          REGISTER_BIT(PINC,1)
#define GYRO_YAW            REGISTER_BIT(PINC,0)
#define GYRO_ROLL_DIR       REGISTER_BIT(DDRC,2)
#define GYRO_PITCH_DIR      REGISTER_BIT(DDRC,1)
#define GYRO_YAW_DIR        REGISTER_BIT(DDRC,0)

#define GAIN_ROLL           REGISTER_BIT(PINC,3)
#define GAIN_PITCH          REGISTER_BIT(PINC,4)
#define GAIN_YAW            REGISTER_BIT(PINC,5)
#define GAIN_ROLL_DIR       REGISTER_BIT(DDRC,3)
#define GAIN_PITCH_DIR      REGISTER_BIT(DDRC,4)
#define GAIN_YAW_DIR        REGISTER_BIT(DDRC,5)

#define GYRO_YAW_ADC_CH     0
#define GYRO_PITCH_ADC_CH   1
#define GYRO_ROLL_ADC_CH    2
#define GAIN_ROLL_ADC_CH    3
#define GAIN_PITCH_ADC_CH   4
#define GAIN_YAW_ADC_CH     5
#define ADC_CHANNELS_MASK	(\
                                _BV(GYRO_YAW_ADC_CH) |      \
                                _BV(GYRO_PITCH_ADC_CH) |    \
                                _BV(GYRO_ROLL_ADC_CH) |     \
                                _BV(GAIN_ROLL_ADC_CH) |     \
                                _BV(GAIN_PITCH_ADC_CH) |    \
                                _BV(GAIN_YAW_ADC_CH)        \
                            )

/* WARNING: PORTB and PORTD are only currently supported for this.
 *  It's easy to add such support but performance will suffer a bit */
#define M1_IS_PORTB         1
#define M1_IS_PORTD         0
#define M2_IS_PORTB         1
#define M2_IS_PORTD         0
#define M3_IS_PORTB         1
#define M3_IS_PORTD         0
#define M4_IS_PORTB         0
#define M4_IS_PORTD         1
#if !defined(ATTITUDE_SENSOR) && !defined(RX_CONFIG)
#define M5_IS_PORTB         0
#define M5_IS_PORTD         1
#define M6_IS_PORTB         0
#define M6_IS_PORTD         1
#endif
#define M1                  _BV(2)
#define M2                  _BV(1)
#define M3                  _BV(0)
#define M4                  _BV(7)
#if !defined(ATTITUDE_SENSOR) && !defined(RX_CONFIG)
#define M5                  _BV(6)
#define M6                  _BV(5)
#endif
#define M1_DIR              REGISTER_BIT(DDRB,2)
#define M2_DIR              REGISTER_BIT(DDRB,1)
#define M3_DIR              REGISTER_BIT(DDRB,0)
#define M4_DIR              REGISTER_BIT(DDRD,7)
#if !defined(ATTITUDE_SENSOR) && !defined(RX_CONFIG)
#define M5_DIR              REGISTER_BIT(DDRD,6)
#define M6_DIR              REGISTER_BIT(DDRD,5)
#endif

#define LED                 REGISTER_BIT(PORTB,6)
#define LED_DIR             REGISTER_BIT(DDRB,6)

#if defined(ATTITUDE_SENSOR) && !defined(RX_CONFIG)
#define ATT_TRIG            REGISTER_BIT(PORTD, 6)
#define ATT_ECHO            REGISTER_BIT(PIND, 5)
#define ATT_TRIG_DIR        REGISTER_BIT(DDRD, 6)
#define ATT_ECHO_DIR        REGISTER_BIT(DDRD, 5)
#endif

#endif //IO_CFG_H
