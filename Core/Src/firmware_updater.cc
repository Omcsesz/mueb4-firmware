/**
 * This file contains firmware updater functionality.
 * The file is located at the end of flash, separated from main program area.
 * @file
 * @author Zsombor Bodnár
 */

#include <socket.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"
#include "network.h"
#include "wiznet_callbacs.h"

extern std::uint32_t main_program_pages[];
extern std::uint32_t firmware_updater_start[];

/**
 * Manages firmware update process.
 * @Note At this point the device should have an IP address, and all peripherals
 * should be initialized.
 */
extern "C" [[maybe_unused]] void FirmwareUpdater() {
  // Prevent hard fault, no Interrupt vector table after flash erase.
  __disable_irq();

  // Register W5500 callback functions
  reg_wizchip_cs_cbfunc(CsSel, CsDesel);
  reg_wizchip_spi_cbfunc(SpiRb, SpiWb);
  reg_wizchip_spiburst_cbfunc(SpiRBurst, SpiWBurst);

  // For program and erase operations on the Flash memory (write/erase), the
  // internal RC oscillator (HSI) must be ON.
  RCC_OscInitTypeDef RCC_OscInitStruct = {
      .OscillatorType = RCC_OSCILLATORTYPE_HSI, .HSIState = RCC_HSI_ON};
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    HAL_NVIC_SystemReset();
  }

restart_update:
  if (socket(Network::kFirmwareUpdaterSocket, Sn_MR_TCP,
             Network::kFirmwareUpdaterPort,
             0x00u) != Network::kFirmwareUpdaterSocket) {
    goto restart_update;
  }

  if (listen(Network::kFirmwareUpdaterSocket) != SOCK_OK) {
    goto restart_update;
  }

  std::uint8_t status;
  do {
    getsockopt(Network::kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (status != SOCK_ESTABLISHED);

  /* The function HAL_FLASH_Unlock() should be called before to unlock the FLASH
   * interface
   */
  if (HAL_FLASH_Unlock() != HAL_OK) {
    disconnect(Network::kFirmwareUpdaterSocket);
    close(Network::kFirmwareUpdaterSocket);
    goto restart_update;
  }

  // FLASH should be previously erased before new programming
  FLASH_EraseInitTypeDef pEraseInit = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = FLASH_BASE,
      .NbPages = reinterpret_cast<std::uint32_t>(main_program_pages)};
  std::uint32_t PageError;
  if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) {
    disconnect(Network::kFirmwareUpdaterSocket);
    close(Network::kFirmwareUpdaterSocket);
    goto restart_update;
  }

  // Write flash page by page
  std::int32_t received_size;
  std::uint32_t base_address{FLASH_BASE};
  do {
    std::array<std::uint8_t, FLASH_PAGE_SIZE> flash_page_buffer{};
    std::uint32_t *flash_page_buffer_p{
        reinterpret_cast<std::uint32_t *>(flash_page_buffer.data())};
    received_size = recv(Network::kFirmwareUpdaterSocket,
                         flash_page_buffer.data(), FLASH_PAGE_SIZE);
    if (received_size > 0u) {
      // Overwrite protection
      if (base_address + received_size >=
          reinterpret_cast<std::uint32_t>(firmware_updater_start)) {
        // At this point no return to main program, need to restart update
        disconnect(Network::kFirmwareUpdaterSocket);
        close(Network::kFirmwareUpdaterSocket);
        goto restart_update;
      }

      for (std::size_t i{0u}; i < received_size / 4u; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_address + i * 4u,
                              flash_page_buffer_p[i]) != HAL_OK) {
          disconnect(Network::kFirmwareUpdaterSocket);
          close(Network::kFirmwareUpdaterSocket);
          goto restart_update;
        }
      }

      /* Handle odd received_size, write last byte
       * This should not happen because of alignment
       */
      if (received_size % 2u != 0u) {
        std::uint32_t last_byte = received_size - 1u;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              base_address + last_byte,
                              flash_page_buffer[last_byte]) != HAL_OK) {
          disconnect(Network::kFirmwareUpdaterSocket);
          close(Network::kFirmwareUpdaterSocket);
          goto restart_update;
        }
      }

      base_address += received_size;
    } else {
      disconnect(Network::kFirmwareUpdaterSocket);
      close(Network::kFirmwareUpdaterSocket);
      goto restart_update;
    }

    getsockopt(Network::kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (getSn_RX_RSR(Network::kFirmwareUpdaterSocket) != 0u ||
           status != SOCK_CLOSE_WAIT);

  send(Network::kFirmwareUpdaterSocket, (std::uint8_t *)"FLASH_OK", 8u);
  disconnect(Network::kFirmwareUpdaterSocket);
  close(Network::kFirmwareUpdaterSocket);

  /* The function HAL_FLASH_Lock() should be called after to lock the FLASH
   * interface
   */
  HAL_FLASH_Lock();
  HAL_NVIC_SystemReset();
}
