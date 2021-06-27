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
extern std::uintptr_t _firmware_updater_start[];

namespace {
SPI_HandleTypeDef hspi1;

/// Firmware updater port number.
constexpr std::uint16_t kFirmwareUpdaterPort{50002u};

/// Socket number for firmware updater.
constexpr std::uint8_t kFirmwareUpdaterSocket{3u};
}  // namespace

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

/**
 * Manages firmware update process.
 * @Note At this point the device should have an IP address, and all peripherals
 * should be initialized.
 */
extern "C" void FirmwareUpdater() {
  // Prevent hard fault, no Interrupt vector table after flash erase.
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    HAL_NVIC_SystemReset();
  }

restart_update:
  if (socket(kFirmwareUpdaterSocket, Sn_MR_TCP, kFirmwareUpdaterPort, 0x00) !=
      kFirmwareUpdaterSocket) {
    HAL_NVIC_SystemReset();
  }

  if (listen(kFirmwareUpdaterSocket) != SOCK_OK) {
    HAL_NVIC_SystemReset();
  }

  std::uint8_t status;
  do {
    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (status != SOCK_ESTABLISHED);

  // The function HAL_FLASH_Unlock() should be called before to unlock the FLASH
  // interface
  if (HAL_FLASH_Unlock() != HAL_OK) {
    disconnect(kFirmwareUpdaterSocket);
    HAL_NVIC_SystemReset();
  }

  // FLASH should be previously erased before new programming
  FLASH_EraseInitTypeDef pEraseInit = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = FLASH_BASE,
      .NbPages = reinterpret_cast<std::uint32_t>(_main_program_pages)};
  std::uint32_t PageError;
  if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) {
    // If this fails we can't do much
    disconnect(kFirmwareUpdaterSocket);
    return;
  }

  // Write flash page by page
  std::int32_t recv_size{0};
  std::uint32_t base_addr{FLASH_BASE};
  do {
    std::array<std::uint8_t, FLASH_PAGE_SIZE> buffer{};
    std::uint32_t *buffer_p{reinterpret_cast<std::uint32_t *>(buffer.data())};
    recv_size = recv(kFirmwareUpdaterSocket, buffer.data(), FLASH_PAGE_SIZE);

    // Overwrite protection
    if (base_addr + recv_size >=
        reinterpret_cast<std::uint32_t>(_firmware_updater_start)) {
      // At this point no return to main program, need to restart update
      disconnect(kFirmwareUpdaterSocket);
      goto restart_update;
    }

    if (recv_size > 0) {
      for (std::size_t i = 0; i < recv_size / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_addr + i * 4,
                              buffer_p[i]) != HAL_OK) {
          // If this fails we can't do much
          disconnect(kFirmwareUpdaterSocket);
          return;
        }
      }

      // Handle odd recv_size, write last byte
      // This should not be called because of alignment
      if (recv_size % 2 != 0) {
        std::int32_t last_byte = recv_size - 1;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr + last_byte,
                              buffer[last_byte]) != HAL_OK) {
          // If this fails we can't do much
          disconnect(kFirmwareUpdaterSocket);
          return;
        }
      }

      base_addr += recv_size;
    } else {
      // Restart update
      disconnect(kFirmwareUpdaterSocket);
      goto restart_update;
    }

    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (getSn_RX_RSR(kFirmwareUpdaterSocket) != 0 ||
           status != SOCK_CLOSE_WAIT);

  send(kFirmwareUpdaterSocket, (std::uint8_t *)"FLASH_OK", 8u);
  disconnect(kFirmwareUpdaterSocket);
  close(kFirmwareUpdaterSocket);

  // The function HAL_FLASH_Lock() should be called after to lock the FLASH
  // interface
  // Should not fail
  HAL_FLASH_Lock();
  HAL_NVIC_SystemReset();
}
