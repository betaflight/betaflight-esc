#include "include.h"

// from https://www.rcgroups.com/forums/showatt.php?attachmentid=8524039&d=1450424877
/*
        KISS ESC TELEMETRY PROTOCOL
        ---------------------------
        One transmission will have 10 times 8-bit bytes sent with 115200 baud and 3.6V.
        Byte 0: Temperature
        Byte 1: Voltage high byte
        Byte 2: Voltage low byte
        Byte 3: Current high byte
        Byte 4: Current low byte
        Byte 5: Consumption high byte
        Byte 6: Consumption low byte
        Byte 7: Rpm high byte
        Byte 8: Rpm low byte
        Byte 9: 8-bit CRC
*/

typedef struct {
    int8_t temperature;// Temperature (resolution 1°C)
    int16_t voltage;// Voltage (resolution 0.01V)
    int16_t current;// Current (resolution 0.01A)
    int16_t consumption;// Consumption (resolution 1mAh)
    int16_t rpm;// Electrical Rpm (resolution 100Rpm)
} escSensorData_t;

static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return crc_u;
}

static uint8_t calculateCrc8(const uint8_t *buf, const uint8_t bufLen)
{
    uint8_t crc = 0;
    for (int i = 0; i < bufLen; i++) {
        crc = updateCrc8(buf[i], crc);
    }

    return crc;
}

#define TELEMETRY_FRAME_SIZE 10
static uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };

static void motor_telemetry_package(escSensorData_t *escSensorData)
{
    // temp
    telemetryBuffer[0] = escSensorData->temperature;

    // voltage
    telemetryBuffer[1] = escSensorData->voltage >> 8;
    telemetryBuffer[2] = escSensorData->voltage & 0xFF;

    // current
    telemetryBuffer[3] = escSensorData->current >> 8;
    telemetryBuffer[4] = escSensorData->current & 0xFF;

    // consumption
    telemetryBuffer[5] = escSensorData->consumption >> 8;
    telemetryBuffer[6] = escSensorData->consumption & 0xFF;

    // 2pole rpm
    telemetryBuffer[7] = escSensorData->rpm >> 8;
    telemetryBuffer[8] = escSensorData->rpm & 0xFF;

    // crc
    telemetryBuffer[9] = calculateCrc8(telemetryBuffer, 9);

    for(uint8_t i = 0; i < TELEMETRY_FRAME_SIZE; i++) {
        serialWrite(telemetryBuffer[i]);
    }
}

void motor_telemetry_thread(void)
{
    static escSensorData_t escSensorData;

    escSensorData.temperature = motor_adc_get_temperature();
    escSensorData.voltage = motor_adc_get_voltage()*100;
    escSensorData.current = motor_adc_get_current()*100;
    escSensorData.consumption = 0;
    escSensorData.rpm = 0;

    motor_telemetry_package(&escSensorData);
}
