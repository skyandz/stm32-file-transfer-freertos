#ifndef STM_CRC32_H
#define STM_CRC32_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Calculate CRC32 MPEG2 (STM32 Hardware CRC compatible)
 * @param crc Initial CRC value (use 0xFFFFFFFF for start)
 * @param buffer Pointer to data buffer
 * @param size Size of data in bytes
 * @return Calculated CRC32 value
 * 
 * @note This implements the same algorithm as STM32 hardware CRC peripheral
 *       Polynomial: 0x04C11DB7 (CRC-32/MPEG-2)
 *       Initial value: 0xFFFFFFFF
 *       No final XOR
 * 
 * @example
 *   uint32_t crc = 0xFFFFFFFF;
 *   crc = crc32(crc, data, len);
 */
uint32_t crc32(uint32_t crc, const uint8_t* buffer, size_t size);

/**
 * @brief CRC record structure for storing file CRC in crc.txt
 */
typedef struct {
    char filename[64];
    uint32_t crc;
} crc_record_t;

#endif // STM_CRC32_H
