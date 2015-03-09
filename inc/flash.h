#ifndef FLASH_H_GG
#define FLASH_H_GG

#include <stdint.h>

#define ADDR_FLASH_GLOBAL_PARAMS 0x080E0000

#define FLASH_OK 0
#define FLASH_ERROR_ERASE 1
#define FLASH_ERROR_WRITE 2

/* ------------------------------------- *
 *  Read, Write and Erase functions      *
 * ------------------------------------- */
 /**
  * @brief  Reads an uint32 buffer from the internal flash at a given address
  * @param  address: Start address of the flash area to read
  * @param  buffer: Address where to store the read buffer (user is responsible to allocate memory
  * @param  buffer_size: Size of the buffer to read in bytes
  * @retval FLASH Status: always returns FLASH_OK
  */
 uint8_t flash_read_buffer_uint32(uint32_t address, uint32_t *buffer, uint16_t buffer_size);

 /**
  * @brief  Reads a float buffer from the internal flash at a given address
  * @param  address: Start address of the flash area to read
  * @param  buffer: Address where to store the read buffer (user is responsible to allocate memory
  * @param  buffer_size: Size of the buffer to read in bytes
  * @retval FLASH Status: always returns FLASH_OK
  */
 uint8_t flash_read_buffer_float(uint32_t address, float *buffer, uint16_t buffer_size);

 /**
  * @brief  Writes a uint32 buffer to the internal flash at a given address
  *         (You need to erase before writing!)
  * @note   This function requires a device voltage between 2.7V and 3.6V.
  * @param  address: Start address of the flash area to write
  * @param  buffer: Address of the buffer to write
  * @param  buffer_size: Size of the buffer to write in bytes
  * @retval FLASH Status: The returned value can be: FLASH_OK or FLASH_ERROR_WRITE.
  */ 
 uint8_t flash_write_buffer_uint32(uint32_t address, uint32_t *buffer, uint16_t buffer_size);
 
  /**
  * @brief  Writes a float buffer to the internal flash at a given address
  *         (You need to erase before writing!)
  * @note   This function requires a device voltage between 2.7V and 3.6V.
  * @param  address: Start address of the flash area to write
  * @param  buffer: Address of the buffer to write
  * @param  buffer_size: Size of the buffer to write in bytes
  * @retval FLASH Status: The returned value can be: FLASH_OK or FLASH_ERROR_WRITE.
  */ 
 uint8_t flash_write_buffer_float(uint32_t address, float *buffer, uint16_t buffer_size);


  /**
  * @brief  Erases one sector of the internal flash
  * @note   This function requires a device voltage between 2.7V and 3.6V.
  * @param  address: an address in the sector to erase
  * @retval FLASH Status: The returned value can be: FLASH_OK or FLASH_ERROR_ERASE.
  */ 
uint8_t flash_erase_sector(uint32_t address);

#endif /* FLASH_H_ */
