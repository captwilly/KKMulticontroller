#ifndef SETTINGS_H
#define SETTINGS_H

#include "config.h"

/*** BEGIN TYPES ***/
// eeProm data structure
struct SETTINGS_S {
  uint8_t setup;                    // Byte to identify if already setup
  uint8_t RollGyroDirection;
  uint8_t PitchGyroDirection;
  uint8_t YawGyroDirection;
};
/*** END TYPES ***/

/*** BEGIN PROTOTYPES ***/
void settingsRead(struct SETTINGS_S *settings);
void settingsWrite(struct SETTINGS_S *settings);
void settingsClearAll(void);
/*** END PROTOTYPES ***/

#endif
