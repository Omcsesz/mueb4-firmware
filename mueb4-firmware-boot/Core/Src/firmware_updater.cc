/**
 * This file contains firmware updater functionality.
 * The file is located at the end of flash, separated from main program area.
 * @file
 * @author Zsombor Bodnár
 */

#include <dhcp.h>
#include <socket.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "i2c.h"
#include "main.h"
#include "wiznet_callbacs.h"

extern std::uint32_t _app_address[];
extern std::uint32_t _app_pages[];

namespace {
const std::uint32_t kBaseAddress{reinterpret_cast<std::uint32_t>(_app_address)};
const std::uint32_t kPages{reinterpret_cast<std::uint32_t>(_app_pages)};
constexpr std::uint16_t kFirmwareUpdaterPort{50002u};
constexpr std::uint16_t kEui48MacStartAddress{0xFAu};
constexpr std::uint8_t kDhcpSocketSocket{0u};
constexpr std::uint8_t kFirmwareUpdaterSocket{1u};
constexpr std::uint8_t kEepromAddress{0b10100001u};
}  // namespace

extern "C" {
bool firmware_update_enabled{false};
std::uint8_t firmware_update_timeout{0u};

/**
 * Manages firmware update process.
 * @Note At this point the device should have an IP address, and all peripherals
 * should be initialized.
 */
void FirmwareUpdater() {
  firmware_update_enabled = true;

  // -- Do not remove --
  // Hardware reset W5500
  HAL_GPIO_WritePin(W5500_RSTN_GPIO_Port, W5500_RSTN_Pin, GPIO_PIN_RESET);
  // RESET should be held low at least 500 us for W5500
  HAL_Delay(1u);
  HAL_GPIO_WritePin(W5500_RSTN_GPIO_Port, W5500_RSTN_Pin, GPIO_PIN_SET);
  // RSTn to internal PLOCK (PLL Lock)
  HAL_Delay(1u);
  // -- Do not remove --

  // Register W5500 callback functions
  reg_wizchip_cris_cbfunc(CrisEn, CrisEx);
  reg_wizchip_cs_cbfunc(CsSel, CsDesel);
  reg_wizchip_spi_cbfunc(SpiRb, SpiWb);
  reg_wizchip_spiburst_cbfunc(SpiRBurst, SpiWBurst);

  // DHCP, firmware updater rx/tx sizes
  std::array<std::uint8_t, 8u> txsize{2u, 2u};
  std::array<std::uint8_t, 8u> rxsize{2u, 8u};

  // This includes soft reset
  wizchip_init(txsize.data(), rxsize.data());

  // Gets MAC address from EEPROM.
  // The device uses Microchip 24AA02E48T-I/OT EEPROM
  std::array<std::uint8_t, 6> mac{0u};

  HAL_I2C_Mem_Read(&hi2c2, kEepromAddress, kEui48MacStartAddress,
                   I2C_MEMADD_SIZE_8BIT, mac.data(), 6u, HAL_MAX_DELAY);
  setSHAR(mac.data());

  // Set all capable, Auto-negotiation enabled
  wiz_PhyConf_t phyconf{PHY_CONFBY_SW, PHY_MODE_AUTONEGO};
  wizphy_setphyconf(&phyconf);

  std::array<std::uint8_t, 576u> dhcp_rx_buffer{0u};

  // DHCP 1s timer located in interrupts.cc
  DHCP_init(kDhcpSocketSocket, dhcp_rx_buffer.data());

  // For program and erase operations on the Flash memory (write/erase), the
  // internal RC oscillator (HSI) must be ON.
  if (RCC_OscInitTypeDef RCC_OscInitStruct = {.OscillatorType =
                                                  RCC_OSCILLATORTYPE_HSI,
                                              .HSIState = RCC_HSI_ON};
      HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    HAL_NVIC_SystemReset();
  }

restart_update:
  while (true) {
    if (wizphy_getphylink() == PHY_LINK_ON) {
      HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_SET);

      if (std::uint8_t dhcp_status = DHCP_run();
          dhcp_status != DHCP_IP_ASSIGN && dhcp_status != DHCP_IP_CHANGED &&
          dhcp_status != DHCP_IP_LEASED) {
        HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
        continue;
      }

      HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);
      break;
    } else {
      HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
      DhcpRebind();
    }
  }

  if (socket(kFirmwareUpdaterSocket, Sn_MR_TCP, kFirmwareUpdaterPort, 0x00u) !=
      kFirmwareUpdaterSocket) {
    goto restart_update;
  }

  // Enable TCP keep alive
  setSn_KPALVTR(kFirmwareUpdaterSocket, 1u);

  if (listen(kFirmwareUpdaterSocket) != SOCK_OK) {
    goto restart_update;
  }

  std::uint8_t status;

  firmware_update_timeout = 0u;  // Start timeout measurement
  while (true) {
    if (firmware_update_timeout == 3u) {
      HAL_NVIC_SystemReset();
    }

    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
    if (status == SOCK_ESTABLISHED) {
      break;
    } else if (status == SOCK_CLOSED) {
      close(kFirmwareUpdaterSocket);
      goto restart_update;
    }
  }

  /* The function HAL_FLASH_Unlock() should be called before to unlock the FLASH
   * interface
   */
  if (HAL_FLASH_Unlock() != HAL_OK) {
    close(kFirmwareUpdaterSocket);
    goto restart_update;
  }

  // FLASH should be previously erased before new programming
  FLASH_EraseInitTypeDef pEraseInit{FLASH_TYPEERASE_PAGES, kBaseAddress,
                                    kPages};
  if (std::uint32_t PageError;
      HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) {
    close(kFirmwareUpdaterSocket);
    goto restart_update;
  }

  // Write flash page by page
  std::int32_t received_size;
  std::uint32_t base_address{kBaseAddress};
  do {
    // Send dummy packet to generate keep alive
    send(kFirmwareUpdaterSocket, (std::uint8_t *)"!", 2u);

    std::array<std::uint8_t, FLASH_PAGE_SIZE> flash_page_buffer{};
    const std::uint32_t *flash_page_buffer_p{
        reinterpret_cast<std::uint32_t *>(flash_page_buffer.data())};

    received_size =
        recv(kFirmwareUpdaterSocket, flash_page_buffer.data(), FLASH_PAGE_SIZE);
    if (received_size < 0) {
      goto restart_update;
    }

    // Overwrite protection
    if (base_address + received_size >= kBaseAddress + kPages * 1024u) {
      // At this point no return to main program, need to restart update
      close(kFirmwareUpdaterSocket);
      goto restart_update;
    }

    std::size_t iterations = received_size / 4u;
    for (std::size_t i{0u}; i < iterations || (iterations == 0 && i < 1); i++) {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_address + i * 4u,
                            flash_page_buffer_p[i]) != HAL_OK) {
        close(kFirmwareUpdaterSocket);
        goto restart_update;
      }
    }

    base_address += received_size;
    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (getSn_RX_RSR(kFirmwareUpdaterSocket) != 0u ||
           status != SOCK_CLOSE_WAIT);

  send(kFirmwareUpdaterSocket, (std::uint8_t *)"FLASH_OK", 9u);
  disconnect(kFirmwareUpdaterSocket);
  close(kFirmwareUpdaterSocket);

  /* The function HAL_FLASH_Lock() should be called after to lock the FLASH
   * interface
   */
  HAL_FLASH_Lock();
  HAL_NVIC_SystemReset();
}
}
