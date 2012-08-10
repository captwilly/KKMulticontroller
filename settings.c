#include "settings.h"

#include "gyros.h"
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/crc16.h>
#include <string.h>

/*** BEGIN DEFINITIONS ***/
/* This magic number is written as CRC when generating EEPROM contents
 *  on first boot system will rewrite that with proper CRC value */
#define EEPROM_SETTINGS_MAGIC	    0x0152B002UL
#define EEPROM_SETTINGS_VERSION     1

// Default settings
#define DEFAULT_SETTINGS_INITIALIZER                                        \
            .crc =                  EEPROM_SETTINGS_MAGIC,                  \
            .settings = {                                                   \
                .Version =              EEPROM_SETTINGS_VERSION,            \
                                                                            \
                .RollGyroDirection =    GYRO_REVERSED,                      \
                .PitchGyroDirection =   GYRO_REVERSED,                      \
                .YawGyroDirection =     GYRO_NORMAL,                        \
                                                                            \
                .RxRollZero =           0,                                  \
                .RxPitchZero =          0,                                  \
                .RxYawZero =            0,                                  \
                .RxCollectiveZero =     -500,                               \
                                                                            \
                .RxRollMin =            -500,                               \
                .RxPitchMin =           -500,                               \
                .RxYawMin =             -500,                               \
                .RxCollectiveMin =      0,                                  \
                                                                            \
                .RxRollMax =            500,                                \
                .RxPitchMax =           500,                                \
                .RxYawMax =             500,                                \
                .RxCollectiveMax =      1000,                               \
                                                                            \
                .Comment = "Default settings, version 1"                    \
            },                                                              \


static struct SETTINGS_STORED_S default_settings
        __attribute__((section(".eeprom"))) = { DEFAULT_SETTINGS_INITIALIZER };
/*** END DEFINITIONS ***/


static uint16_t settingsCalcCRC(struct SETTINGS_S *set) {
    // Cast to avoid compiler warnings
    uint8_t *ptr = (uint8_t*)set;
    uint16_t retval = 0;

    for (uint8_t i = 0; i < sizeof(struct SETTINGS_S); i++) {
        retval = _crc16_update(retval, ptr[i]);
    }

    return retval;
}

void settingsWrite(struct SETTINGS_S *settings) {
    // Buffer for parent structure (with CRC)
    struct SETTINGS_STORED_S to_store;

    // Read and update CRC
    memcpy(&to_store.settings, settings, sizeof(struct SETTINGS_S));
    to_store.crc = settingsCalcCRC(&to_store.settings);

    // Write
    eeprom_update_block(
            &to_store, &default_settings, sizeof(struct SETTINGS_STORED_S));

    // Read back and to check successful write
    struct SETTINGS_STORED_S check;
    eeprom_read_block(
            &check, &default_settings, sizeof(struct SETTINGS_STORED_S));
    if (0 != memcmp(&to_store, &check, sizeof(struct SETTINGS_STORED_S))) {
        // Indicate an error if problem reading
        while (true) {
            LED = 1;
            _delay_ms(1000);
            LED = 0;
            _delay_ms(1000);
        }
    }
}

static void settingsSetDefaults(void) {
    struct SETTINGS_STORED_S to_store = {DEFAULT_SETTINGS_INITIALIZER};
    to_store.crc = settingsCalcCRC(&to_store.settings);
    eeprom_update_block(
            &to_store, &default_settings, sizeof(struct SETTINGS_S));
}

void settingsRead(struct SETTINGS_S *settings) {
    // Read settings structure (without CRC)
    eeprom_read_block(
            settings, &default_settings.settings, sizeof(struct SETTINGS_S));

    // Read stored CRC
    uint32_t crc_stored = eeprom_read_dword(&default_settings.crc);

    /* Check: if we have MAGIC we just write settings back (writing procedure
     *  will correct CRC) */
    if (EEPROM_SETTINGS_MAGIC == crc_stored) {
        settingsWrite(settings);
    } else {
        uint16_t crc_calc = settingsCalcCRC(settings);
        // In case CRC is bad, reset to defaults
        if (crc_calc != crc_stored) {
            settingsSetDefaults();
            eeprom_read_block(settings, &default_settings.settings,
                    sizeof(struct SETTINGS_S));
        }

        // Check for proper settings version
        if (settings->Version != EEPROM_SETTINGS_VERSION) {
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
