#pragma once

#define EEPROM_CONF_VERSION 1

void writeEEPROM(void);
void readEEPROM(void);
bool isEEPROMContentValid(void);
