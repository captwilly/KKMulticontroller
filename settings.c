#include "settings.h"

#include "gyros.h"
#include <avr/eeprom.h>
#include <util/delay.h>

/*** BEGIN DEFINITIONS ***/
#define EEPROM_SETTINGS_MAGIC	    42

// Default settings
#define DEFAULT_SETTINGS_INITIALIZER    \
            .setup =                EEPROM_SETTINGS_MAGIC,                  \
                                                                            \
            .RollGyroDirection =    GYRO_REVERSED,                          \
            .PitchGyroDirection =   GYRO_REVERSED,                          \
            .YawGyroDirection =     GYRO_NORMAL,                            \
                                                                            \
            .RxRollZero =           0,                                      \
            .RxPitchZero =          0,                                      \
            .RxYawZero =            0,                                      \


static struct SETTINGS_S default_settings
        __attribute__((section(".eeprom"))) = { DEFAULT_SETTINGS_INITIALIZER };
/*** END DEFINITIONS ***/

void settingsWrite(struct SETTINGS_S *settings) {
    settings->setup = EEPROM_SETTINGS_MAGIC;
    eeprom_update_block(settings, &default_settings, sizeof(struct SETTINGS_S));
}

static void settingsSetDefaults(void) {
    struct SETTINGS_S settings = { DEFAULT_SETTINGS_INITIALIZER };
    settingsWrite(&settings);
}

void settingsRead(struct SETTINGS_S *settings) {
    eeprom_read_block(settings, &default_settings, sizeof(struct SETTINGS_S));
    if (EEPROM_SETTINGS_MAGIC != settings->setup) {
        settingsSetDefaults();
        eeprom_read_block(settings, &default_settings,
                sizeof(struct SETTINGS_S));
        if (EEPROM_SETTINGS_MAGIC != settings->setup) {
            while (true) {
                LED = 1;
                _delay_ms(1000);
                LED = 0;
                _delay_ms(1000);
            }
        }
    }
}

void settingsClearAll() {
    for (uint8_t i = 0; i < 5; i++) {
        LED = 1;
        _delay_ms(25);
        LED = 0;
        _delay_ms(25);
    }

    settingsSetDefaults();
    while (true);
}
