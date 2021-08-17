/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#include "network.h"

#include <dhcp.h>
#include <socket.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iterator>

#include "crc.h"
#include "i2c.h"
#include "main.h"
#include "panel.h"
#include "tim.h"
#include "version.h"
#include "wiznet_callbacs.h"

///@{
/// Defined in linker script.
extern std::uint32_t firmware_updater_start[];
extern std::uint32_t firmware_updater_pages[];
extern std::uint32_t main_program_size[];
extern std::uint32_t flash_end[];
///@}

namespace {
/// Stores main program flash size in words.
// NOLINTNEXTLINE
const std::uint32_t kMainProgramSize =
    reinterpret_cast<std::uint32_t>(main_program_size) / 4u;

std::uint16_t animation_buffer_offset;

/**
 * Stores network information.
 * Contains MAC address, Source IP, Subnet mask etc.
 */
wiz_NetInfo net_info{};

inline void UpdateIp() {
  wizchip_getnetinfo(&net_info);
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);

  auto level_number = static_cast<std::uint8_t>(18u - net_info.ip[2]);
  auto room_number = static_cast<std::uint8_t>(net_info.ip[3] - 5u);
  animation_buffer_offset =
      static_cast<std::uint16_t>((level_number * 96u + room_number * 6u) + 2u);
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

    if (getSn_RX_RSR(kAnimationSocket) > 0u && net_info.ip[2] != 0u &&
        net_info.ip[3] != 0u) {
      HandleAnimationProtocol();
    }
  } else {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    DhcpRebind();
  }
}

Network::Network() {
  // -- Do not remove --
  // Hardware reset W5500
  HAL_GPIO_WritePin(W5500_RSTn_GPIO_Port, W5500_RSTn_Pin, GPIO_PIN_RESET);
  // RESET should be held low at least 500 us for W5500
  HAL_Delay(1u);
  HAL_GPIO_WritePin(W5500_RSTn_GPIO_Port, W5500_RSTn_Pin, GPIO_PIN_SET);
  // RSTn to internal PLOCK (PLL Lock)
  HAL_Delay(1u);
  // -- Do not remove --

  // Register W5500 callback functions
  reg_wizchip_cris_cbfunc(CrisEn, CrisEx);
  reg_wizchip_cs_cbfunc(CsSel, CsDesel);
  reg_wizchip_spi_cbfunc(SpiRb, SpiWb);
  reg_wizchip_spiburst_cbfunc(SpiRBurst, SpiWBurst);
  reg_dhcp_cbfunc(IpAssign, IpUpdate, IpConflict);

  // DHCP, command protocol, broadcast protocol, firmware socket rx/tx sizes
  std::array<std::uint8_t, 8u> txsize{1u, 1u, 1u, 1u};
  std::array<std::uint8_t, 8u> rxsize{1u, 1u, 4u, 8u};
  // This includes soft reset
  wizchip_init(txsize.data(), rxsize.data());

  // Gets MAC address from EEPROM.
  // The device uses Microchip 24AA02E48T-I/OT EEPROM
  HAL_I2C_Mem_Read(&hi2c2, kEepromAddress, kEui48MacStartAddress,
                   I2C_MEMADD_SIZE_8BIT, net_info.mac, 6u, HAL_MAX_DELAY);
  setSHAR(net_info.mac);

  // Set all capable, Auto-negotiation enabled
  wiz_PhyConf_t phyconf{PHY_CONFBY_SW, PHY_MODE_AUTONEGO};
  wizphy_setphyconf(&phyconf);

  // DHCP 1s timer located in interrupts.cc
  DHCP_init(kDhcpSocket, dhcp_rx_buffer_.data());

  socket(kCommandSocket, Sn_MR_UDP, kCommandSocketPort, 0x00u);

  // Enable multicast and join multicast group
  setsockopt(
      kAnimationSocket, SO_DESTIP,
      const_cast<std::uint8_t *>(kAnimationProtocolMulticastAddress.data()));
  setsockopt(kAnimationSocket, SO_DESTPORT,
             const_cast<std::uint16_t *>(&kAnimationSocketPort));
  socket(kAnimationSocket, Sn_MR_UDP, kAnimationSocketPort,
         SF_MULTI_ENABLE | SF_IGMP_VER2);
}

template <std::size_t N>
std::tuple<std::int32_t, std::array<std::uint8_t, N>,
           std::array<std::uint8_t, 4u>, std::uint16_t>
Network::CheckIpAddress(const std::uint8_t &socket_number) {
  std::array<std::uint8_t, N> buffer{};
  std::array<std::uint8_t, 4u> server_address{};
  std::uint16_t server_port;

  std::int32_t size{recvfrom(socket_number, buffer.data(),
                             static_cast<std::uint16_t>(buffer.size()),
                             server_address.data(), &server_port)};
  if (size < 0 || (!std::equal(std::begin(net_info.gw), std::end(net_info.gw),
                               server_address.begin()) &&
                   server_address[2] != 0u)) {
    return std::make_tuple(-1, std::move(buffer), server_address, server_port);
  }

  return std::make_tuple(size, std::move(buffer), server_address, server_port);
}

void Network::HandleAnimationProtocol() {
  auto [size, buffer, server_address,
        server_port]{CheckIpAddress<kMtu>(kAnimationSocket)};
  // Handle too small and incorrect packages
  if (buffer[0] != kAnimationProtocolVersion ||
      (buffer[0] == kAnimationProtocolVersion &&
       size < kAnimationProtocolSize)) {
    return;
  }

  Panel::SetInternalAnimation(false);

  auto buffer_begin{buffer.begin() + animation_buffer_offset};
  Panel::ColorData colors{};
  auto colors_begin{colors.begin()};

  for (auto [i, bytes] = std::tuple(buffer_begin, 0u);; i++, bytes++) {
    if (bytes == 3u || bytes == 9u) {
      // Jump to next row
      i = i - 3u + 48u;
    } else if (bytes == 6u) {
      // Send left panel data
      Panel::GetPanel(Panel::Side::LEFT).SendColorData(colors);
      i -= 48u;
      colors_begin = colors.begin();
    } else if (bytes == 12u) {
      // Send right panel data
      Panel::GetPanel(Panel::Side::RIGHT).SendColorData(colors);
      break;
    }

    // Uncompress 2 color data from 1 byte into 2 bytes
    *colors_begin = (*i & 0xF0u) >> 4u;
    colors_begin++;
    *colors_begin = *i & 0x0Fu;
    colors_begin++;
  }
}

void Network::HandleCommandProtocol() {
  auto [size, buffer, server_address,
        server_port]{CheckIpAddress<kCommandProtocolMaxSize>(kCommandSocket)};
  // Handle too small and incorrect packages
  if (buffer[0] != 'S' || buffer[1] != 'E' || buffer[2] != 'M' || size < 4) {
    return;
  }

  /* When the 5th byte is set to 1 it means we're sending a broadcast command to
   * only one device
   * Can be used when the device doesn't have an IP address
   */
  if (buffer[4] == 1u) {
    // Return when the MAC address doesn't match
    if (!std::equal(std::begin(net_info.mac), std::end(net_info.mac),
                    buffer.begin() + 5u)) {
      return;
    }

    // If the device's IP address is 0.0.0.0 use broadcast target address
    if (std::all_of(std::begin(net_info.ip), std::end(net_info.ip),
                    [](const std::uint8_t &i) { return i == 0u; })) {
      server_address.fill(0xFFu);
    }
  }

  switch (static_cast<Command>(buffer[3])) {
      // Mutable commands
    case Command::kDisablePanels:
      Panel::DisableAll();
      break;
    case Command::kSetWhiteBalance: {
      Panel::WhiteBalanceData white_balance{};
      std::copy_n(buffer.begin() + 11u, white_balance.size(),
                  white_balance.begin());
      Panel::SendWhiteBalanceToAll(white_balance);
      break;
    }
    case Command::kUseInternalAnimation:
      Panel::SetInternalAnimation(true);
      break;
    case Command::kUseExternalAnimation:
      Panel::SetInternalAnimation(false);
      break;
    case Command::kSwapPanels:
      Panel::Swap();
      break;
    case Command::kBlank:
      Panel::BlankAll();
      break;
    case Command::kReset:
      HAL_NVIC_SystemReset();
      break;
    case Command::kStartFirmwareUpdate: {
      /* Jump to firmware updater program segment, no going back
       * Modifies PC
       */
      void *f{firmware_updater_start};
      goto *f;
      break;
    }
    case Command::kFlashFirmwareUpdater:
      FlashFirmwareUpdater();
      break;
    // Immutable commands
    case Command::kPing:
      sendto(kCommandSocket, (std::uint8_t *)"pong", 4u, server_address.data(),
             server_port);
      break;
    case Command::kGetStatus: {
      const char *format{
          // clang-format off
        "MUEB FW: %s\n"
        "MUEB MAC: %x:%x:%x:%x:%x:%x\n"
        "Internal animation: %s\n"
        "Left panel state: %#x\n"
        "Right panel state: %#x\n"
        "SEM & KSZK forever",
          // clang-format on
      };
      const auto status_string_size = std::snprintf(
          nullptr, 0u, format, mueb_version, net_info.mac[0], net_info.mac[1],
          net_info.mac[2], net_info.mac[3], net_info.mac[4], net_info.mac[5],
          (Panel::internal_animation_enabled() ? "on" : "off"),
          Panel::GetPanel(Panel::Side::LEFT).state(),
          Panel::GetPanel(Panel::Side::RIGHT).state());
      char status_string[status_string_size + 1];

      std::snprintf(status_string, sizeof status_string, format, mueb_version,
                    net_info.mac[0], net_info.mac[1], net_info.mac[2],
                    net_info.mac[3], net_info.mac[4], net_info.mac[5],
                    (Panel::internal_animation_enabled() ? "on" : "off"),
                    Panel::GetPanel(Panel::Side::LEFT).state(),
                    Panel::GetPanel(Panel::Side::RIGHT).state());

      sendto(kCommandSocket, reinterpret_cast<std::uint8_t *>(status_string),
             static_cast<std::uint16_t>(status_string_size),
             server_address.data(), server_port);
      break;
    }
    case Command::kGetMac: {
      std::array<char, 18u> mac{};
      sendto(kCommandSocket, reinterpret_cast<std::uint8_t *>(mac.data()),
             static_cast<std::uint16_t>(std::snprintf(
                 mac.data(), mac.size(), "%x:%x:%x:%x:%x:%x", net_info.mac[0],
                 net_info.mac[1], net_info.mac[2], net_info.mac[3],
                 net_info.mac[4], net_info.mac[5])),
             server_address.data(), server_port);
      break;
    }
    case Command::kGetFirmwareChecksum: {
      /* __REV, for endianness fix, little to big endian
       * ~, negate(crc XOR 0xFFFFFFFF) for standard CRC32
       */
      auto crc{__REV(~HAL_CRC_Calculate(
          &hcrc, reinterpret_cast<std::uint32_t *>(FLASH_BASE),
          kMainProgramSize))};

      sendto(kCommandSocket, (std::uint8_t *)&crc, sizeof(crc),
             server_address.data(), server_port);
      break;
    }
    case Command::kGetFirmwareUpdaterChecksum: {
      if (!firmware_updater_size_) {
        return;
      }

      // firmware updater size in words
      const std::uint16_t firmware_updater_size{
          static_cast<const std::uint16_t>(firmware_updater_size_ / 4u)};
      auto crc{__REV(~HAL_CRC_Calculate(
          &hcrc, reinterpret_cast<std::uint32_t *>(firmware_updater_start),
          firmware_updater_size))};

      sendto(kCommandSocket, (std::uint8_t *)&crc, sizeof(crc),
             server_address.data(), server_port);
      break;
    }
  }
}

void Network::FlashFirmwareUpdater() {
  firmware_updater_size_ = 0;

  // For program and erase operations on the Flash memory (write/erase), the
  // internal RC oscillator (HSI) must be ON.
  RCC_OscInitTypeDef RCC_OscInitStruct = {
      .OscillatorType = RCC_OSCILLATORTYPE_HSI, .HSIState = RCC_HSI_ON};
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    return;
  }

  if (socket(kFirmwareUpdaterSocket, Sn_MR_TCP, kFirmwareUpdaterPort, 0x00u) !=
      kFirmwareUpdaterSocket) {
    return;
  }

  if (listen(kFirmwareUpdaterSocket) != SOCK_OK) {
    return;
  }

  std::uint8_t status;
  do {
    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (status != SOCK_ESTABLISHED);

  // The function HAL_FLASH_Unlock() should be called before to unlock the
  // FLASH interface
  if (HAL_FLASH_Unlock() != HAL_OK) {
    disconnect(kFirmwareUpdaterSocket);
    close(kFirmwareUpdaterSocket);
    return;
  }

  // FLASH should be previously erased before new programming
  FLASH_EraseInitTypeDef pEraseInit = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = reinterpret_cast<std::uint32_t>(firmware_updater_start),
      .NbPages = reinterpret_cast<std::uint32_t>(firmware_updater_pages)};
  std::uint32_t PageError;
  if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) {
    disconnect(kFirmwareUpdaterSocket);
    close(kFirmwareUpdaterSocket);
    return;
  }

  // Write flash page by page
  std::int32_t received_size;
  std::uint32_t base_address{
      reinterpret_cast<std::uint32_t>(firmware_updater_start)};
  do {
    std::array<std::uint8_t, FLASH_PAGE_SIZE> flash_page_buffer{};
    std::uint32_t *flash_page_buffer_p{
        reinterpret_cast<std::uint32_t *>(flash_page_buffer.data())};
    received_size =
        recv(kFirmwareUpdaterSocket, flash_page_buffer.data(), FLASH_PAGE_SIZE);

    // Overwrite protection
    if (base_address + received_size >=
        reinterpret_cast<std::uint32_t>(flash_end)) {
      disconnect(kFirmwareUpdaterSocket);
      close(kFirmwareUpdaterSocket);
      return;
    }

    if (received_size > 0u) {
      for (std::size_t i{0u}; i < received_size / 4u; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_address + i * 4u,
                              flash_page_buffer_p[i]) != HAL_OK) {
          disconnect(kFirmwareUpdaterSocket);
          close(kFirmwareUpdaterSocket);
          return;
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
          disconnect(kFirmwareUpdaterSocket);
          close(kFirmwareUpdaterSocket);
          return;
        }
      }

      base_address += received_size;
      firmware_updater_size_ += received_size;
    } else {
      disconnect(kFirmwareUpdaterSocket);
      close(kFirmwareUpdaterSocket);
      return;
    }

    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (getSn_RX_RSR(kFirmwareUpdaterSocket) != 0u ||
           status != SOCK_CLOSE_WAIT);

  send(kFirmwareUpdaterSocket, (std::uint8_t *)"FLASH_OK", 8u);
  disconnect(kFirmwareUpdaterSocket);
  close(kFirmwareUpdaterSocket);

  /* The function HAL_FLASH_Lock() should be called after to lock the FLASH
   * interface
   */
  HAL_FLASH_Lock();
}
