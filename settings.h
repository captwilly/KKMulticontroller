#ifndef SETTINGS_H
#define SETTINGS_H

#include "common.h"

/*** BEGIN TYPES ***/
// Settings structure
struct SETTINGS_S {
    // TODO: split to separate structures - one per module which uses settings
    uint8_t Version;

    uint8_t RollGyroDirection;
    uint8_t PitchGyroDirection;
    uint8_t YawGyroDirection;

    int16_t RxRollZero;
    int16_t RxPitchZero;
    int16_t RxYawZero;
    int16_t RxCollectiveZero;

    int16_t RxRollMin;
    int16_t RxPitchMin;
    int16_t RxYawMin;
    int16_t RxCollectiveMin;

    int16_t RxRollMax;
    int16_t RxPitchMax;
    int16_t RxYawMax;
    int16_t RxCollectiveMax;

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
