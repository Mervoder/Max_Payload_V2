/*
 * w25q.h
 *
 *  Created on: Apr 27, 2024
 *      Author: oguzk
 */


#ifndef INC_W25Q_H_
#define INC_W25Q_H_
#include "stm32f4xx.h"

extern SPI_HandleTypeDef hspi1;

#define W25Q_CHIP_SELECT_LOW()     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define W25Q_CHIP_SELECT_HIGH()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)

#define CMD_WRITE_ENABLE          0x06
#define CMD_READ_DATA             0x03
#define CMD_PAGE_PROGRAM          0x02
#define CMD_READ_STATUS_REG       0x05
#define CMD_SECTOR_ERASE    0x20  // Sektör silme komutu

void W25Q_WriteEnable(void);
uint8_t W25Q_ReadStatusRegister(void);
void W25Q_WaitForWriteEnd(void);
void W25Q_WriteData(uint32_t address, uint8_t* data, uint16_t size) ;
void W25Q_ReadData(uint32_t address, uint8_t* buffer, uint16_t size);
void W25Q_SectorErase(uint32_t address);


#endif /* INC_W25Q_H_ */
