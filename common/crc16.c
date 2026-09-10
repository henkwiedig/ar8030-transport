#include "crc16.h"

uint16_t ar8030_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < len; i++) {
        crc = (uint16_t)(crc ^ ((uint16_t)data[i] << 8));
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }

    return crc;
}
