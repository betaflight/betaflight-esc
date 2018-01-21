#include "include.h"

#include "stm32f0xx_hal_flash.h"

#define FLASH_PAGE_COUNT                64
#define FLASH_TO_RESERVE_FOR_CONFIG     0x800
#define CONFIG_START_FLASH_ADDRESS      (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0;
    const uint8_t *byteOffset;

    for (byteOffset = data; byteOffset < (data + length); byteOffset++)
        checksum ^= *byteOffset;

    return checksum;
}

bool isEEPROMContentValid(void)
{
    const master_t *temp = (const master_t *) CONFIG_START_FLASH_ADDRESS;
    uint8_t checksum = 0;

    if (EEPROM_CONF_VERSION != temp->version) {
        return false;
    }

    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF) {
        return false;
    }

    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0) {
        return false;
    }
    return true;
}

void writeEEPROM(void)
{

    int8_t attemptsRemaining = 3;

    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.chk = 0; 
    masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(master_t));

    HAL_StatusTypeDef status;
    HAL_FLASH_Unlock();
    while (attemptsRemaining--) {
        for (uint32_t wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
            if (wordOffset % FLASH_PAGE_SIZE == 0) {

                FLASH_EraseInitTypeDef eraseInit;
                eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
                eraseInit.PageAddress = CONFIG_START_FLASH_ADDRESS + wordOffset;
                eraseInit.NbPages = 1;

                uint32_t eraseError = 0;

                status = HAL_FLASHEx_Erase(&eraseInit, &eraseError);
                if (status != HAL_OK) {
                    break;
                }
            }
            
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_START_FLASH_ADDRESS + wordOffset, *(uint32_t *) ((char *) &masterConfig + wordOffset));
            if (status != HAL_OK) {
                break;
            }
        }

        if (status == HAL_OK) {
            break;
        }
    }

    HAL_FLASH_Lock();

    if (status != HAL_OK || !isEEPROMContentValid()) {

    }
}

void readEEPROM(void)
{
    if (!isEEPROMContentValid()) {

    }

    memcpy(&masterConfig, (char *) CONFIG_START_FLASH_ADDRESS, sizeof(master_t));
}
