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
#include <w5500.h>

#include "gpios.h"
#include "main.h"

SPI_HandleTypeDef hspi1;

/// WIZnet chip select
void cs_sel() {
  reset_gpio(SPI1_NSS);  // ChipSelect to low
}

/// WIZnet chip deselect
void cs_desel() {
  set_gpio(SPI1_NSS);  // ChipSelect to high
}

/// Read byte from WIZnet chip through SPI
uint8_t spi_rbyte(void) {
  uint8_t ret;
  HAL_SPI_Receive(&hspi1, &ret, 1, 255);

  return ret;
}

/// Write byte to WIZnet chip through SPI
void spi_wbyte(uint8_t b) { HAL_SPI_Transmit(&hspi1, &b, 1, 255); }

/// Read burst from WIZnet chip through SPI
void spi_rburst(uint8_t *pBuf, uint16_t len) {
  HAL_SPI_Receive(&hspi1, pBuf, len, 255);
}

/// Write burst to WIZnet chip through SPI
void spi_wburst(uint8_t *pBuf, uint16_t len) {
  HAL_SPI_Transmit(&hspi1, pBuf, len, 255);
}

/// Manages firmware update process.
void firmware_update() {
  __disable_irq();

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  LL_RCC_HSI_Enable();

  while (!LL_RCC_HSI_IsReady())
    ;

  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rbyte, spi_wbyte);
  reg_wizchip_spiburst_cbfunc(spi_rburst, spi_wburst);

  socket(3, Sn_MR_TCP, 1997, 0x00);

  if (listen(3) != SOCK_OK) return;

  if (HAL_FLASH_Unlock() != HAL_OK) return;

  uint8_t status;
  do {
    getsockopt(3, SO_STATUS, &status);
  } while (status != SOCK_ESTABLISHED);

  uint32_t base_addr = FLASH_BASE;

  // Update NbPages when firmware update's flash size changes 64 - 10 = 54
  FLASH_EraseInitTypeDef pEraseInit = {.NbPages = 54,
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
