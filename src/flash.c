
/* Includes ------------------------------------------------------------------*/
#include "flash.h"
#include "settings.h"
#include "stm32f4xx_flash.h"

#define MAX_WRITE_TRIALS  100  /* Number of trials to write to flash */



/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes   bootloader        */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes   application       */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes   application       */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes   application       */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes   application       */    
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes                    */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes                    */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes                    */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes                    */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes                    */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes                   */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes global parameters */


/* private functions */
uint32_t GetSector(uint32_t Address);

 /**
  * @brief  Reads an uint32 buffer from the internal flash at a given address
  * @param  address: Start address of the flash area to read
  * @param  buffer: Address where to store the read buffer (user is responsible to allocate memory
  * @param  buffer_size: Size of the buffer to read in bytes
  * @retval FLASH Status: always returns FLASH_OK
  */
uint8_t flash_read_buffer_uint32(uint32_t address, uint32_t *buffer, uint16_t buffer_size)
{    
    for(uint16_t i = 0; i < buffer_size; i++)
    {
        buffer[i] = *(__IO uint32_t*)(address+i*4);
    }
    return FLASH_OK;
}


 /**
  * @brief  Reads a float buffer from the internal flash at a given address
  * @param  address: Start address of the flash area to read
  * @param  buffer: Address where to store the read buffer (user is responsible to allocate memory
  * @param  buffer_size: Size of the buffer to read in bytes
  * @retval FLASH Status: always returns FLASH_OK
  */
uint8_t flash_read_buffer_float(uint32_t address, float *buffer, uint16_t buffer_size)
{
    return flash_read_buffer_uint32(address, (uint32_t*) buffer, buffer_size);
}


 /**
  * @brief  Writes a uint32 buffer to the internal flash at a given address
  *         (You need to erase before writing!)
  * @note   This function requires a device voltage between 2.7V and 3.6V.
  * @note   The values are read back and compared, if problems are detected,
  *         we retry MAX_WRITE_TRIALS times and then abort
  * @param  address: Start address of the flash area to write
  * @param  buffer: Address of the buffer to write
  * @param  buffer_size: Size of the buffer to write in bytes
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */ 
uint8_t flash_write_buffer_uint32(uint32_t address, uint32_t *buffer, uint16_t buffer_size)
{
    FLASH_Unlock();
    
    /* we write everything and than check what we wrote */
    uint32_t i = 0;
    uint32_t trials = 0;
    uint8_t write_error;
    FLASH_Status status = FLASH_COMPLETE;
    do{
        write_error = 0;
        trials ++;
        /* Write parameters to flash (WE ASSUME VOLTAGE OF 2.7V to 3.6V) */
        for(; i < buffer_size; i++)
        {
            status = FLASH_ProgramWord(address+i*4, buffer[i]);
            if(status != FLASH_COMPLETE)
            {
                write_error = 1;
                break;
            }
        }
        
        /* if we tried to often, we return FLASH_ERROR_OPERATION */
        if(trials > MAX_WRITE_TRIALS)
        {
            return status;
        }
        
        /* if we had a write error, try again for same index */
        if(write_error)
        {
            continue;
        }
        
        /* Read flash and compare */
        for(i = 0; i < buffer_size; i++)
        {
            if(*(__IO float*)(address+i*4) != buffer[i])
            {
                write_error = 1;
                break;
            }
        }
    }while(write_error);
    
    /* we lock the flash to prevent accidental writing */
    FLASH_Lock();
    
    return FLASH_COMPLETE;
}


  /**
  * @brief  Writes a float buffer to the internal flash at a given address
  *         (You need to erase before writing!)
  * @note   This function requires a device voltage between 2.7V and 3.6V.
  * @param  address: Start address of the flash area to write
  * @param  buffer: Address of the buffer to write
  * @param  buffer_size: Size of the buffer to write in bytes
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */ 
uint8_t flash_write_buffer_float(uint32_t address, float *buffer, uint16_t buffer_size)
{
    return flash_write_buffer_uint32(address, (uint32_t*) buffer, buffer_size);
}



  /**
  * @brief  Erases one sector of the internal flash
  * @note   This function requires a device voltage between 2.7V and 3.6V.
  * @note   This function locks the flash after erasing
  * @param  address: an address in the sector to erase
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */ 
uint8_t flash_erase_sector(uint32_t address)
{
    FLASH_Unlock();
    return FLASH_EraseSector(GetSector(address), VoltageRange_3);
    FLASH_Lock();
}


/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
    sector = FLASH_Sector_11;  
  }

  return sector;
}
