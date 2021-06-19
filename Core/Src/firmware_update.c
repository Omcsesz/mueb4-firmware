/**
 * This file contains firmware update functionality.
 * The file is located at the end of flash, separated from main program area.
 * @file
 * @author Zsombor Bodnár
 */

#include <socket.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f0xx.h>
#include <wizchip_conf.h>

#include "main.h"

SPI_HandleTypeDef hspi1 = {.Instance = SPI1,
                           .Init.Mode = SPI_MODE_MASTER,
                           .Init.Direction = SPI_DIRECTION_2LINES,
                           .Init.DataSize = SPI_DATASIZE_8BIT,
                           .Init.CLKPolarity = SPI_POLARITY_LOW,
                           .Init.CLKPhase = SPI_PHASE_1EDGE,
                           .Init.NSS = SPI_NSS_SOFT,
                           .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
                           .Init.FirstBit = SPI_FIRSTBIT_MSB,
                           .Init.TIMode = SPI_TIMODE_DISABLE,
                           .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
                           .Init.CRCPolynomial = 7,
                           .Init.CRCLength = SPI_CRC_LENGTH_DATASIZE,
                           .Init.NSSPMode = SPI_NSS_PULSE_DISABLE};

/// WIZnet chip select
void CsSel() {
  // ChipSelect to low
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
}

/// WIZnet chip deselect
void CsDesel() {
  // ChipSelect to high
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

/// Read byte from WIZnet chip through SPI
uint8_t SpiRb() {
  uint8_t pData;
  HAL_SPI_Receive(&hspi1, &pData, 1u, HAL_MAX_DELAY);

  return pData;
}

/// Write byte to WIZnet chip through SPI
void SpiWb(uint8_t pData) {
  HAL_SPI_Transmit(&hspi1, &pData, 1u, HAL_MAX_DELAY);
}

/// Read burst from WIZnet chip through SPI
void SpiRBurst(uint8_t *pData, uint16_t Size) {
  HAL_SPI_Receive(&hspi1, pData, Size, HAL_MAX_DELAY);
}

/// Write burst to WIZnet chip through SPI
void SpiWBurst(uint8_t *pData, uint16_t Size) {
  HAL_SPI_Transmit(&hspi1, pData, Size, HAL_MAX_DELAY);
}

/// Manages firmware update process.
void FirmwareUpdate() {
  __disable_irq();

  // For program and erase operations on the Flash memory (write/erase), the
  // internal RC oscillator (HSI) must be ON.
  RCC_OscInitTypeDef RCC_OscInitStruct = {
      .OscillatorType = RCC_OSCILLATORTYPE_HSI, .HSIState = RCC_HSI_ON};
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  reg_wizchip_cs_cbfunc(CsSel, CsDesel);
  reg_wizchip_spi_cbfunc(SpiRb, SpiWb);
  reg_wizchip_spiburst_cbfunc(SpiRBurst, SpiWBurst);

  socket(3, Sn_MR_TCP, 1997, 0x00);

  if (listen(3) != SOCK_OK) return;

  if (HAL_FLASH_Unlock() != HAL_OK) return;

  uint8_t status;
  do {
    getsockopt(3, SO_STATUS, &status);
  } while (status != SOCK_ESTABLISHED);

  uint32_t base_addr = FLASH_BASE;

  // Update NbPages when firmware update's flash size changes 64 - 12 = 52
  FLASH_EraseInitTypeDef pEraseInit = {.NbPages = 52,
                                       .PageAddress = base_addr,
                                       .TypeErase = FLASH_TYPEERASE_PAGES};
  uint32_t PageError;
  if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) return;

  int32_t recvsize = 0;
  do {
    uint8_t buf[FLASH_PAGE_SIZE] = {};

    if ((recvsize = recv(3, buf, FLASH_PAGE_SIZE)) > 0) {
      int i;
      for (i = 0; i < recvsize - 1; i += 2) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr + i,
                              buf[i + 1] << 8 | buf[i]) != HAL_OK)
          return;
      }

      if (recvsize % 2 != 0) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr + i,
                              buf[i]) != HAL_OK)
          return;
      }

      base_addr += recvsize;
    }

    getsockopt(3, SO_STATUS, &status);
  } while (status != SOCK_CLOSED);

  HAL_NVIC_SystemReset();
}
