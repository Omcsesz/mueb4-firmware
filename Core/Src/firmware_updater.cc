/**
 * This file contains firmware updater functionality.
 * The file is located at the end of flash, separated from main program area.
 * @file
 * @author Zsombor Bodnár
 */

#include <socket.h>
#include <wizchip_conf.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

extern std::uintptr_t _main_program_pages[];
extern std::uintptr_t _main_program_start[];

SPI_HandleTypeDef hspi1;

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
std::uint8_t SpiRb() {
  std::uint8_t pData;
  HAL_SPI_Receive(&hspi1, &pData, 1u, HAL_MAX_DELAY);

  return pData;
}

/// Write byte to WIZnet chip through SPI
void SpiWb(std::uint8_t pData) {
  HAL_SPI_Transmit(&hspi1, &pData, 1u, HAL_MAX_DELAY);
}

/// Read burst from WIZnet chip through SPI
void SpiRBurst(std::uint8_t *pData, std::uint16_t Size) {
  HAL_SPI_Receive(&hspi1, pData, Size, HAL_MAX_DELAY);
}

/// Write burst to WIZnet chip through SPI
void SpiWBurst(std::uint8_t *pData, std::uint16_t Size) {
  HAL_SPI_Transmit(&hspi1, pData, Size, HAL_MAX_DELAY);
}

/// Manages firmware update process.
extern "C" void FirmwareUpdater() {
  __disable_irq();

  // Register W5500 callback functions
  reg_wizchip_cs_cbfunc(CsSel, CsDesel);
  reg_wizchip_spi_cbfunc(SpiRb, SpiWb);
  reg_wizchip_spiburst_cbfunc(SpiRBurst, SpiWBurst);

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

  // For program and erase operations on the Flash memory (write/erase), the
  // internal RC oscillator (HSI) must be ON.
  RCC_OscInitTypeDef RCC_OscInitStruct = {
      .OscillatorType = RCC_OSCILLATORTYPE_HSI, .HSIState = RCC_HSI_ON};
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  if (socket(3, Sn_MR_TCP, 1997, 0x00) != 3) {
    return;
  }

  if (listen(3) != SOCK_OK) {
    return;
  }

  std::uint8_t status;
  do {
    getsockopt(3, SO_STATUS, &status);
  } while (status != SOCK_ESTABLISHED);

  // The function HAL_FLASH_Unlock() should be called before to unlock the FLASH
  // interface
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return;
  }

  // FLASH should be previously erased before new programming
  FLASH_EraseInitTypeDef pEraseInit = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = FLASH_BASE,
      .NbPages = reinterpret_cast<std::uint32_t>(_main_program_pages)};
  std::uint32_t PageError;
  if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) {
    return;
  }

  // Write flash page by page
  std::int32_t recv_size{0};
  std::uint32_t base_addr{FLASH_BASE};
  do {
    std::uint8_t buffer[FLASH_PAGE_SIZE]{};
    std::uint16_t *buffer_p{reinterpret_cast<std::uint16_t *>(buffer)};

    if ((recv_size = recv(3, buffer, FLASH_PAGE_SIZE)) > 0) {
      for (std::size_t i = 0; i < recv_size / 2; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr + i * 2,
                              buffer_p[i]) != HAL_OK) {
          return;
        }
      }

      if (recv_size % 2 != 0) {
        std::int32_t last_byte = recv_size - 1;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr + last_byte,
                              buffer[last_byte]) != HAL_OK) {
          return;
        }
      }

      base_addr += recv_size;
    } else {
      return;
    }

    getsockopt(3, SO_STATUS, &status);
  } while (getSn_RX_RSR(3) != 0 || status != SOCK_CLOSE_WAIT);

  disconnect(3);
  close(3);

  HAL_NVIC_SystemReset();
}
