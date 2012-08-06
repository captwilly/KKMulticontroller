#ifndef SETTINGS_H
#define SETTINGS_H

#include "common.h"

/*** BEGIN TYPES ***/
// Settings structure
struct SETTINGS_S {
    uint8_t Version;

    uint8_t RollGyroDirection;
    uint8_t PitchGyroDirection;
    uint8_t YawGyroDirection;

    int16_t RxRollZero;
    int16_t RxPitchZero;
    int16_t RxYawZero;

    uint8_t Comment[100];
};

// Structure actually saved to EEPROM (SETTINGS_S + CRC)
struct SETTINGS_STORED_S {
    uint32_t crc;
    struct SETTINGS_S settings;
};
/*** END TYPES ***/

/*** BEGIN PROTOTYPES ***/
void settingsRead(struct SETTINGS_S *settings);
void settingsWrite(struct SETTINGS_S *settings);
void settingsClearAll(void);
/*** END PROTOTYPES ***/

#endif
