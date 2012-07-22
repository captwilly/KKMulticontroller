#include "settings.h"

#include <avr/eeprom.h>
#include "gyros.h"

/*** BEGIN DEFINITIONS ***/
#define EEPROM_DATA_START_POS 0      // Settings save offset in eeprom
/*** END DEFINITIONS ***/

void settingsWrite(struct SETTINGS_S *settings)
{
	settings->setup = 42;
	eeprom_update_block((void*)settings, (void *)EEPROM_DATA_START_POS,
			sizeof(struct SETTINGS_S));
}

static void settingsSetDefaults(void)
{
	struct SETTINGS_S settings;

	settings.RollGyroDirection  = GYRO_REVERSED;
	settings.PitchGyroDirection  = GYRO_REVERSED;
	settings.YawGyroDirection    = GYRO_NORMAL;
	settings.setup = 42;

	settingsWrite(&settings);
}

void settingsRead(struct SETTINGS_S *settings){
	eeprom_read_block(settings, (void *)EEPROM_DATA_START_POS, sizeof(struct SETTINGS_S));
	if(42 != settings->setup){
		settingsSetDefaults();
		settingsRead(settings);
	}
}

void settingsClearAll()
{
  for(uint8_t i = 0;i < 5;i++) {
    LED = 1;
    _delay_ms(25);
    LED = 0;
    _delay_ms(25);
  }

  settingsSetDefaults();
  while(1);
}
