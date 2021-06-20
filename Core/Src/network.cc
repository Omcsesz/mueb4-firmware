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
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

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
extern std::uintptr_t _firmware_updater_handler[];
extern std::uintptr_t _main_program_end[];
///@}

namespace {
// Calculate main program flash size in words
const std::uint32_t kMainProgramSize =
    (reinterpret_cast<std::uintptr_t>(_main_program_end) - FLASH_BASE) / 4u;

/// Stores the level number where the device is located
std::uint8_t level_number{0u};
/// Stores the room number where the device is located
std::uint8_t room_number{0u};

/**
 * Stores network information.
 * Contains MAC address, Source IP, Subnet mask etc.
 */
wiz_NetInfo net_info;

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
}

void IpAssign() {
  default_ip_assign();

  UpdateIp();
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);
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
  // Hard-reset W5500
  HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_RESET);
  // Min reset cycle 500 us
  HAL_Delay(1u);
  HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_SET);
  // PLL lock 1 ms max (refer datasheet)
  HAL_Delay(1u);

  // Register W5500 callback functions
  reg_wizchip_cris_cbfunc(CrisEn, CrisEx);
  reg_wizchip_cs_cbfunc(CsSel, CsDesel);
  reg_wizchip_spi_cbfunc(SpiRb, SpiWb);
  reg_wizchip_spiburst_cbfunc(SpiRBurst, SpiWBurst);
  reg_dhcp_cbfunc(IpAssign, IpUpdate, IpConflict);

  // DHCP, command, broadcast protocol, firmware
  std::array<std::uint8_t, 8> txsize{1u, 1u, 1u, 1u};
  std::array<std::uint8_t, 8> rxsize{1u, 1u, 4u, 8u};
  // This includes soft reset
  wizchip_init(txsize.data(), rxsize.data());

  // Gets MAC address from EEPROM.
  // The device uses Microchip 24AA02E48T-I/OT EEPROM
  HAL_I2C_Mem_Read(&hi2c2, kEepromAddress, kEui48StartAddress,
                   I2C_MEMADD_SIZE_8BIT, net_info.mac, 6u, HAL_MAX_DELAY);

  setSHAR(net_info.mac);
  net_info.dhcp = dhcp_mode::NETINFO_DHCP;

  // Set all capable, Auto-negotiation enabled
  wiz_PhyConf_t phyconf{PHY_CONFBY_SW, PHY_MODE_AUTONEGO};
  wizphy_setphyconf(&phyconf);

  // DHCP 1s timer located in stm32f0xx_it.c
  DHCP_init(kDhcpSocket, dhcp_rx_buffer_);

  socket(kCommandSocket, Sn_MR_UDP, kCommandSocketPort, 0x00u);
  socket(kBroadcastSocket, Sn_MR_UDP, kBroadcastSocketPort, 0x00u);
}

Network &Network::Instance() {
  static Network net;

  return net;
}

void Network::Step() {
  if (wizphy_getphylink() == PHY_LINK_ON) {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_SET);

    if (getSn_RX_RSR(kCommandSocket) > 0u) {
      FetchRemoteCommand();
    }

    // do DHCP task
    switch (DHCP_run()) {
      case DHCP_IP_ASSIGN:
      case DHCP_IP_CHANGED:
      case DHCP_IP_LEASED: {
        if (getSn_RX_RSR(kBroadcastSocket) > 0u && level_number != 0u &&
            room_number != 0u) {
          FetchFrameBroadcastProtocol();
        }
      } break;
      case DHCP_FAILED:
        HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
        break;
    }
  } else {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    DHCP_rebind();
  }
}

void Network::FetchFrameBroadcastProtocol() {
  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_SET);
  Panel::internal_animation_enabled_ = false;

  std::uint8_t buffer[1500]{};
  std::uint8_t server_address[4]{};
  std::uint16_t server_port;
  auto size{recvfrom(kBroadcastSocket, buffer, sizeof(buffer), server_address,
                     &server_port)};
  if (buffer[0] != 0x02u || (buffer[0u] == 0x02u && size < 1250u)) return;

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

void Network::FetchRemoteCommand() {
  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_SET);

  std::array<std::uint8_t, 32u> buffer{};
  std::uint8_t server_address[4]{};
  std::uint16_t server_port;

  auto size{recvfrom(kCommandSocket, buffer.data(), sizeof(buffer),
                     server_address, &server_port)};

  // Handle too small and incorrect packages
  if (buffer[0] != 'S' || buffer[1] != 'E' || buffer[2] != 'M' || size < 4u) {
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
      Panel::internal_animation_enabled_ = false;
      break;
    case Command::kUseInternalAnim:
      Panel::internal_animation_enabled_ = true;
      break;
    case Command::kBlank:
      Panel::LeftPanel().Blank();
      Panel::RightPanel().Blank();
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
      NVIC_SystemReset();
      break;
    case Command::kGetStatus: {
      char status_string[256]{};
      sendto(kCommandSocket, reinterpret_cast<std::uint8_t *>(status_string),
             std::snprintf(status_string, sizeof(status_string),
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
                           Panel::internal_animation_enabled_ ? "on" : "off",
                           getSn_RX_RSR(kCommandSocket),
                           getSn_RX_RSR(kBroadcastSocket)),
             server_address, server_port);
      break;
    }
    case Command::kGetMac:
      char mac[18];
      std::snprintf(mac, sizeof(mac), "%x:%x:%x:%x:%x:%x", net_info.mac[0],
                    net_info.mac[1], net_info.mac[2], net_info.mac[3],
                    net_info.mac[4], net_info.mac[5]);
      sendto(kCommandSocket, reinterpret_cast<std::uint8_t *>(mac), 17u,
             server_address, server_port);
      break;
    case Command::kFlushSocketBuffers:
      FlushBuffers();
      break;
    case Command::kPing:
      sendto(kCommandSocket, (std::uint8_t *)"pong", 4u, server_address,
             server_port);
      break;
    case Command::kStartFirmwareUpdate: {
      // Generate jump instruction
      void *f{_firmware_updater_handler};
      goto *f;
      break;
    }
    case Command::kGetFirmwareChecksum: {
      auto crc = HAL_CRC_Calculate(&hcrc, (std::uint32_t *)FLASH_BASE,
                                   kMainProgramSize);
      /* __REV for endianness fix
       * negate(crc XOR 0xFFFFFFFF) for standard CRC32
       */
      crc = __REV(~crc);

      sendto(kCommandSocket, (std::uint8_t *)&crc, sizeof(crc), server_address,
             server_port);
      break;
    }
    case Command::kSwapPanels:
      Panel::SwapPanels();
      break;
    case Command::kSetWhitebalance: {
      auto white_balance{Panel::kWhiteBalance};
      std::copy_n(buffer.begin() + 11u, white_balance.size(),
                  white_balance.begin());
      Panel::GetPanel(Panel::LEFT).SetWhitebalance(white_balance);
      Panel::GetPanel(Panel::RIGHT).SetWhitebalance(white_balance);
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

  if ((size = getSn_RX_RSR(kBroadcastSocket))) {
    wiz_recv_ignore(kBroadcastSocket, size);
    setSn_CR(kBroadcastSocket, Sn_CR_RECV);
    while (getSn_CR(kBroadcastSocket)) {
    }
  }
}
