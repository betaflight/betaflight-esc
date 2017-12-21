#include "include.h"

// 每页的大小,每页为1KB
#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)

// 总页数，STM32F051K8U6为小容量产品,共64KB的FLASH,共64页
#define FLASH_PAGE_COUNT 64
// 用于储存配置信息(预留2KB共2048字节作为配置信息)
#define FLASH_TO_RESERVE_FOR_CONFIG 0x800
// 配置参数往Flash写入的起始地址(起始地址0x08000000  + (总内存大小 -  EEPROM配置文件大小))    [0x0801F800  -  0x0801FFF]
#define CONFIG_START_FLASH_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))

/**
 * 计算校验位
 */
static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0;
    const uint8_t *byteOffset;

    for (byteOffset = data; byteOffset < (data + length); byteOffset++)
        checksum ^= *byteOffset;

    return checksum;
}

/**
 * 检查EEPROM是否有效
 */
bool isEEPROMContentValid(void)
{
    const master_t *temp = (const master_t *) CONFIG_START_FLASH_ADDRESS;
    uint8_t checksum = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return false;

    // check size and magic numbers
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return false;

    // verify integrity of temporary copy
    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    // looks good, let's roll!
    return true;
}

/**
 * 写入EEPROM
 */
void writeEEPROM(void)
{
    FLASH_Status status = 0;
    int8_t attemptsRemaining = 3;

    // prepare checksum/version constants
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.chk = 0; // erase checksum before recalculating
    masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(master_t));

    // write it
    /* Unlock the Flash to enable the flash control register access *************/
    FLASH_Unlock();
    while (attemptsRemaining--) {
        /* Clear pending flags (if any) */
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

        for (uint32_t wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
            if (wordOffset % FLASH_PAGE_SIZE == 0) {
                status = FLASH_ErasePage(CONFIG_START_FLASH_ADDRESS + wordOffset);
                if (status != FLASH_COMPLETE) {
                    break;
                }
            }

            status = FLASH_ProgramWord(CONFIG_START_FLASH_ADDRESS + wordOffset, *(uint32_t *) ((char *) &masterConfig + wordOffset));
            if (status != FLASH_COMPLETE) {
                break;
            }
        }

        if (status == FLASH_COMPLETE) {
            break;
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) *********/
    FLASH_Lock();

    // Flash write failed - just die now
    if (status != FLASH_COMPLETE || !isEEPROMContentValid()) {

    }
}

/**
 * 从EEPROM中读取配置信息
 */
void readEEPROM(void)
{
    // Sanity check
    if (!isEEPROMContentValid()) {

    }

    // Read flash 读取FLASH中的配置信息,使用memcpy函数复制
    memcpy(&masterConfig, (char *) CONFIG_START_FLASH_ADDRESS, sizeof(master_t));
}
