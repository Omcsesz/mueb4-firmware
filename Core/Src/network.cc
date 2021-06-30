/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#include "network.h"

#include <dhcp.h>
#include <socket.h>
#include <wizchip_conf.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <utility>

#include "main.h"
#include "panel.h"
#include "version.h"

///@{
/// Defined in main.c
extern CRC_HandleTypeDef hcrc;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c2;
///@}

///@{
/// Defined in linker script
extern std::uintptr_t _firmware_updater_start[];
extern std::uintptr_t _firmware_updater_pages[];
extern std::uintptr_t _main_program_size[];
extern std::uintptr_t _flash_end[];
///@}

namespace {
/// Calculate main program flash size in words
const std::uint32_t kMainProgramSize =
    reinterpret_cast<std::uint32_t>(_main_program_size) / 4u;

/// Stores the level number where the device is located
std::uint8_t level_number{0u};
/// Stores the room number where the device is located
std::uint8_t room_number{0u};

/**
 * Stores network information.
 * Contains MAC address, Source IP, Subnet mask etc.
 */
wiz_NetInfo net_info{};

/// WIZnet critical enter
void CrisEn() { __disable_irq(); }

/// WIZnet critical exit
void CrisEx() { __enable_irq(); }

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

inline void UpdateIp() {
  wizchip_getnetinfo(&net_info);
  level_number = net_info.ip[2];
  room_number = net_info.ip[3];
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);
}

void IpAssign() {
  default_ip_assign();
  UpdateIp();
}

void IpUpdate() {
  default_ip_update();
  UpdateIp();
}

void IpConflict() {
  default_ip_conflict();
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
}
}  // namespace

Network::Network() {
  // Hardware reset W5500
  HAL_GPIO_WritePin(W5500_RSTn_GPIO_Port, W5500_RSTn_Pin, GPIO_PIN_RESET);
  // RESET should be held low at least 500 us for W5500
  HAL_Delay(1u);
  HAL_GPIO_WritePin(W5500_RSTn_GPIO_Port, W5500_RSTn_Pin, GPIO_PIN_SET);
  // RSTn to internal PLOCK (PLL Lock)
  HAL_Delay(1u);

  // Register W5500 callback functions
  reg_wizchip_cris_cbfunc(CrisEn, CrisEx);
  reg_wizchip_cs_cbfunc(CsSel, CsDesel);
  reg_wizchip_spi_cbfunc(SpiRb, SpiWb);
  reg_wizchip_spiburst_cbfunc(SpiRBurst, SpiWBurst);
  reg_dhcp_cbfunc(IpAssign, IpUpdate, IpConflict);

  // DHCP, command protocol, broadcast protocol, firmware socket rx/tx sizes
  std::array<std::uint8_t, 8> txsize{1u, 1u, 1u, 1u};
  std::array<std::uint8_t, 8> rxsize{1u, 1u, 4u, 8u};
  // This includes soft reset
  wizchip_init(txsize.data(), rxsize.data());

  // Gets MAC address from EEPROM.
  // The device uses Microchip 24AA02E48T-I/OT EEPROM
  HAL_I2C_Mem_Read(&hi2c2, kEepromAddress, kEui48StartAddress,
                   I2C_MEMADD_SIZE_8BIT, net_info.mac, 6u, HAL_MAX_DELAY);
  setSHAR(net_info.mac);

  // Set all capable, Auto-negotiation enabled
  wiz_PhyConf_t phyconf{PHY_CONFBY_SW, PHY_MODE_AUTONEGO};
  wizphy_setphyconf(&phyconf);

  // DHCP 1s timer located in stm32f0xx_it.c
  DHCP_init(kDhcpSocket, dhcp_rx_buffer_.data());

  socket(kCommandSocket, Sn_MR_UDP, kCommandSocketPort, 0x00u);

  // Enable multicast and join multicast group
  setsockopt(
      kAnimationSocket, SO_DESTIP,
      const_cast<std::uint8_t *>(kAnimationSocketMulticastAddress.data()));
  setsockopt(kAnimationSocket, SO_DESTPORT,
             const_cast<std::uint16_t *>(&kAnimationSocketPort));
  socket(kAnimationSocket, Sn_MR_UDP, kAnimationSocketPort, SF_MULTI_ENABLE);
}

Network &Network::Instance() {
  static Network network;

  return network;
}

void Network::Step() {
  if (wizphy_getphylink() == PHY_LINK_ON) {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_SET);
    if (DHCP_run() == DHCP_FAILED) {
      HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    }

    if (getSn_RX_RSR(kCommandSocket) > 0u) {
      HandleCommandProtocol();
    }

    if (getSn_RX_RSR(kAnimationSocket) > 0u && level_number != 0u &&
        room_number != 0u) {
      HandleAnimationProtocol();
    }
  } else {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    DhcpRebind();
  }
}

template <std::size_t N>
std::tuple<std::int32_t, std::array<std::uint8_t, N>,
           std::array<std::uint8_t, 4u>, std::uint16_t>
Network::HandlePacket(const std::uint8_t &socket_number) {
  std::array<std::uint8_t, N> buffer{};
  std::array<std::uint8_t, 4u> server_address{};
  std::uint16_t server_port;
  std::int32_t size{recvfrom(socket_number, buffer.data(), buffer.size(),
                             server_address.data(), &server_port)};

  if (size < 0 || (!std::equal(std::begin(net_info.gw), std::end(net_info.gw),
                               server_address.begin()) &&
                   server_address[2] != 0)) {
    return std::make_tuple(-1, std::move(buffer), std::move(server_address),
                           std::move(server_port));
  }

  return std::make_tuple(std::move(size), std::move(buffer),
                         std::move(server_address), std::move(server_port));
}

void Network::HandleAnimationProtocol() {
  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_SET);
  Panel::SetInternalAnimation(false);

  auto [size, buffer, server_address,
        server_port]{HandlePacket<1500u>(kAnimationSocket)};
  // Handle too small and incorrect packages
  if (buffer[0] != 0x02u || (buffer[0] == 0x02u && size < 1250)) {
    return;
  }

  if (buffer[0] == 0x02u) {
    std::uint32_t base_offset{0u};
    std::size_t running_offset{0u};
    auto left_panel_pixels{Panel::kPixels};
    auto right_panel_pixels{Panel::kPixels};

    /* Read compressed pixels row wise from frame buffer using 2x2 window
     * 2x2 window position is based on room index
     * Data can be read as a 16x26 2D array
     * room row * 32 vertical unit
     * room column * 2 horizontal unit
     * 3 byte per unit
     * 2 byte packet header
     */
    base_offset =
        ((18u - level_number) * 32u + (room_number - 5u) * 2u) * 3u + 2u;

    // First row
    left_panel_pixels[0].red =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    left_panel_pixels[0].green =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    left_panel_pixels[0].blue =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;

    left_panel_pixels[1].red =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    left_panel_pixels[1].green =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    left_panel_pixels[1].blue =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);

    right_panel_pixels[0].red =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    right_panel_pixels[0].green =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    right_panel_pixels[0].blue =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;

    right_panel_pixels[1].red =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    right_panel_pixels[1].green =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    right_panel_pixels[1].blue =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);

    /*
     * Second row
     * Byte per row = 8 room_number per row * 6 byte per window row
     */
    running_offset = 48u;

    left_panel_pixels[2].red =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    left_panel_pixels[2].green =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    left_panel_pixels[2].blue =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;

    left_panel_pixels[3].red =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    left_panel_pixels[3].green =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    left_panel_pixels[3].blue =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);

    right_panel_pixels[2].red =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    right_panel_pixels[2].green =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    right_panel_pixels[2].blue =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;

    right_panel_pixels[3].red =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);
    right_panel_pixels[3].green =
        (buffer[(base_offset + running_offset)] & 0xF0u) >> 4u;
    right_panel_pixels[3].blue =
        (buffer[(base_offset + running_offset++)] & 0x0Fu);

    Panel::GetPanel(Panel::LEFT).SendPixels(left_panel_pixels);
    Panel::GetPanel(Panel::RIGHT).SendPixels(right_panel_pixels);
  }

  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_RESET);
}

void Network::HandleCommandProtocol() {
  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_SET);

  auto [size, buffer, server_address,
        server_port]{HandlePacket<32u>(kCommandSocket)};
  // Handle too small and incorrect packages
  if (buffer[0] != 'S' || buffer[1] != 'E' || buffer[2] != 'M' || size < 4) {
    return;
  }

  /*
   * When the 5th byte is set to 1 it means we're sending a broadcast command to
   * only one device
   * Can be used when the device doesn't have an IP address
   */
  if (buffer[4] == 1u) {
    if (net_info.mac[0] != buffer[5] || net_info.mac[1] != buffer[6] ||
        net_info.mac[2] != buffer[7] || net_info.mac[3] != buffer[8] ||
        net_info.mac[4] != buffer[9] || net_info.mac[5] != buffer[10]) {
      // return when the MAC address doesn't match
      return;
    }

    // If the device's IP is 0.0.0.0 use broadcast target address
    if (!net_info.ip[0] && !net_info.ip[1] && !net_info.ip[2] &&
        !net_info.ip[3])
      server_address[0] = server_address[1] = server_address[2] =
          server_address[3] = 0xFFu;
  }

  switch (buffer[3]) {
    case Command::kUseExternalAnim:
      Panel::SetInternalAnimation(false);
      break;
    case Command::kUseInternalAnim:
      Panel::SetInternalAnimation(true);
      break;
    case Command::kBlank:
      Panel::BlankAll();
      break;
    case Command::kTurn12vOffLeft:
      Panel::GetPanel(Panel::LEFT).SetStatus(Panel::kVcc12vOff);
      break;
    case Command::kTurn12vOffRight:
      Panel::GetPanel(Panel::RIGHT).SetStatus(Panel::kVcc12vOff);
      break;
    case Command::kResetLeftPanel:
      Panel::GetPanel(Panel::LEFT).SetStatus(Panel::kDischargeCaps);
      break;
    case Command::kResetRightPanel:
      Panel::GetPanel(Panel::RIGHT).SetStatus(Panel::kDischargeCaps);
      break;
    case Command::kReboot:
      HAL_NVIC_SystemReset();
      break;
    case Command::kGetStatus: {
      std::array<char, 256> status_string;
      sendto(kCommandSocket,
             reinterpret_cast<std::uint8_t *>(status_string.data()),
             std::snprintf(status_string.data(), status_string.size(),
                           // clang-format off
              "MUEB FW version: %s\n"
              "MUEB MAC: %x:%x:%x:%x:%x:%x\n"
              "Internal animation: %s\n"
              "Command socket buffer: %#x\n"
              "Broadcast socket buffer: %#x\n"
              "SEM forever",
                           // clang-format on
                           mueb_version, net_info.mac[0], net_info.mac[1],
                           net_info.mac[2], net_info.mac[3], net_info.mac[4],
                           net_info.mac[5],
                           Panel::internal_animation_enabled() ? "on" : "off",
                           getSn_RX_RSR(kCommandSocket),
                           getSn_RX_RSR(kAnimationSocket)),
             server_address.data(), server_port);
      break;
    }
    case Command::kGetMac: {
      std::array<char, 18> mac{};
      sendto(kCommandSocket, reinterpret_cast<std::uint8_t *>(mac.data()),
             std::snprintf(mac.data(), mac.size(), "%x:%x:%x:%x:%x:%x",
                           net_info.mac[0], net_info.mac[1], net_info.mac[2],
                           net_info.mac[3], net_info.mac[4], net_info.mac[5]),
             server_address.data(), server_port);
      break;
    }
    case Command::kFlushSocketBuffers:
      FlushBuffers();
      break;
    case Command::kPing:
      sendto(kCommandSocket, (std::uint8_t *)"pong", 4u, server_address.data(),
             server_port);
      break;
    case Command::kStartFirmwareUpdate: {
      // Generate jump instruction, no going back
      void *f{_firmware_updater_start};
      goto *f;
      break;
    }
    case Command::kGetFirmwareChecksum: {
      auto crc{HAL_CRC_Calculate(&hcrc,
                                 reinterpret_cast<std::uint32_t *>(FLASH_BASE),
                                 kMainProgramSize)};
      /* __REV for endianness fix
       * negate(crc XOR 0xFFFFFFFF) for standard CRC32
       */
      crc = __REV(~crc);

      sendto(kCommandSocket, (std::uint8_t *)&crc, sizeof(crc),
             server_address.data(), server_port);
      break;
    }
    case Command::kGetFirmwareUpdaterChecksum: {
      std::uint16_t const firmware_updater_size{
          (buffer[11] << 8u | buffer[12]) / 4u};
      auto crc{HAL_CRC_Calculate(
          &hcrc, reinterpret_cast<std::uint32_t *>(_firmware_updater_start),
          firmware_updater_size)};

      /* __REV for endianness fix
       * negate(crc XOR 0xFFFFFFFF) for standard CRC32
       */
      crc = __REV(~crc);

      sendto(kCommandSocket, (std::uint8_t *)&crc, sizeof(crc),
             server_address.data(), server_port);
      break;
    }
    case Command::kSwapPanels:
      Panel::SwapPanels();
      break;
    case Command::kSetWhitebalance: {
      auto white_balance{Panel::kWhiteBalance};
      std::copy_n(buffer.begin() + 11u, white_balance.size(),
                  white_balance.begin());
      Panel::GetPanel(Panel::LEFT).SendWhitebalance(white_balance);
      Panel::GetPanel(Panel::RIGHT).SendWhitebalance(white_balance);
      break;
    }
    case Command::kFlashFirmwareUpdater: {
      // For program and erase operations on the Flash memory (write/erase), the
      // internal RC oscillator (HSI) must be ON.
      RCC_OscInitTypeDef RCC_OscInitStruct = {
          .OscillatorType = RCC_OSCILLATORTYPE_HSI, .HSIState = RCC_HSI_ON};
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        break;
      }

      if (socket(kFirmwareUpdaterFlasherSocket, Sn_MR_TCP,
                 kFirmwareUpdaterFlasherPort,
                 0x00) != kFirmwareUpdaterFlasherSocket) {
        break;
      }

      if (listen(kFirmwareUpdaterFlasherSocket) != SOCK_OK) {
        break;
      }

      std::uint8_t status;
      do {
        getsockopt(kFirmwareUpdaterFlasherSocket, SO_STATUS, &status);
      } while (status != SOCK_ESTABLISHED);

      // The function HAL_FLASH_Unlock() should be called before to unlock the
      // FLASH interface
      if (HAL_FLASH_Unlock() != HAL_OK) {
        disconnect(kFirmwareUpdaterFlasherSocket);
        close(kFirmwareUpdaterFlasherSocket);
        break;
      }

      // FLASH should be previously erased before new programming
      FLASH_EraseInitTypeDef pEraseInit = {
          .TypeErase = FLASH_TYPEERASE_PAGES,
          .PageAddress =
              reinterpret_cast<std::uint32_t>(_firmware_updater_start),
          .NbPages = reinterpret_cast<std::uint32_t>(_firmware_updater_pages)};
      std::uint32_t PageError;
      if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) {
        // If this fails we can't do much
        disconnect(kFirmwareUpdaterFlasherSocket);
        close(kFirmwareUpdaterFlasherSocket);
        break;
      }

      // Write flash page by page
      std::int32_t recv_size{0};
      std::uint32_t base_addr{
          reinterpret_cast<std::uint32_t>(_firmware_updater_start)};
      do {
        std::array<std::uint8_t, FLASH_PAGE_SIZE> buffer{};
        std::uint32_t *buffer_p{
            reinterpret_cast<std::uint32_t *>(buffer.data())};
        recv_size =
            recv(kFirmwareUpdaterFlasherSocket, buffer.data(), FLASH_PAGE_SIZE);

        // Overwrite protection
        if (base_addr + recv_size >=
            reinterpret_cast<std::uint32_t>(_flash_end)) {
          disconnect(kFirmwareUpdaterFlasherSocket);
          close(kFirmwareUpdaterFlasherSocket);
          break;
        }

        if (recv_size > 0) {
          for (std::size_t i = 0; i < recv_size / 4; i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_addr + i * 4,
                                  buffer_p[i]) != HAL_OK) {
              // If this fails we can't do much
              disconnect(kFirmwareUpdaterFlasherSocket);
              close(kFirmwareUpdaterFlasherSocket);
              break;
            }
          }

          // Handle odd recv_size, write last byte
          // This should not be called because of alignment
          if (recv_size % 2 != 0) {
            std::int32_t last_byte = recv_size - 1;
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_addr + last_byte,
                                  buffer[last_byte]) != HAL_OK) {
              // If this fails we can't do much
              disconnect(kFirmwareUpdaterFlasherSocket);
              close(kFirmwareUpdaterFlasherSocket);
              break;
            }
          }

          base_addr += recv_size;
        } else {
          disconnect(kFirmwareUpdaterFlasherSocket);
          close(kFirmwareUpdaterFlasherSocket);
          break;
        }

        getsockopt(kFirmwareUpdaterFlasherSocket, SO_STATUS, &status);
      } while (getSn_RX_RSR(kFirmwareUpdaterFlasherSocket) != 0 ||
               status != SOCK_CLOSE_WAIT);

      send(kFirmwareUpdaterFlasherSocket, (std::uint8_t *)"FLASH_OK", 8u);
      disconnect(kFirmwareUpdaterFlasherSocket);
      close(kFirmwareUpdaterFlasherSocket);

      // The function HAL_FLASH_Lock() should be called after to lock the FLASH
      // interface
      // Should not fail
      HAL_FLASH_Lock();
      break;
    }
  }

  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_RESET);
}

void Network::FlushBuffers() {
  auto size{getSn_RX_RSR(kCommandSocket)};

  if (size) {
    wiz_recv_ignore(kCommandSocket, getSn_RX_RSR(kCommandSocket));
    setSn_CR(kCommandSocket, Sn_CR_RECV);
    while (getSn_CR(kCommandSocket)) {
    }
  }

  if ((size = getSn_RX_RSR(kAnimationSocket))) {
    wiz_recv_ignore(kAnimationSocket, size);
    setSn_CR(kAnimationSocket, Sn_CR_RECV);
    while (getSn_CR(kAnimationSocket)) {
    }
  }
}
