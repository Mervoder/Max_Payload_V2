/*
 * w25q.c
 *
 *  Created on: Apr 27, 2024
 *      Author: oguzk
 */

#include "w25q.h"

extern SPI_HandleTypeDef hspi1;

void W25Q_WriteEnable(void) {
    uint8_t cmd = CMD_WRITE_ENABLE;
    W25Q_CHIP_SELECT_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    W25Q_CHIP_SELECT_HIGH();
}

uint8_t W25Q_ReadStatusRegister(void) {
    uint8_t cmd = CMD_READ_STATUS_REG;
    uint8_t status;
    W25Q_CHIP_SELECT_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    W25Q_CHIP_SELECT_HIGH();
    return status;
}

void W25Q_WaitForWriteEnd(void) {
    uint8_t status;
    do {
        status = W25Q_ReadStatusRegister();
    } while ((status & 0x01) == 0x01);
}

void W25Q_WriteData(uint32_t address, uint8_t* data, uint16_t size) {
    W25Q_WriteEnable();
    uint8_t cmd[4] = {CMD_PAGE_PROGRAM, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    W25Q_CHIP_SELECT_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    W25Q_CHIP_SELECT_HIGH();
    W25Q_WaitForWriteEnd();
}

void W25Q_ReadData(uint32_t address, uint8_t* buffer, uint16_t size) {
    uint8_t cmd[4] = {CMD_READ_DATA, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    W25Q_CHIP_SELECT_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    W25Q_CHIP_SELECT_HIGH();
}



void W25Q_SectorErase(uint32_t address) {
    W25Q_WriteEnable();  // Yazma izni

    uint8_t cmd[4] = {CMD_SECTOR_ERASE, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    W25Q_CHIP_SELECT_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    W25Q_CHIP_SELECT_HIGH();

    W25Q_WaitForWriteEnd();  // Silme işleminin bitmesini bekle
}

