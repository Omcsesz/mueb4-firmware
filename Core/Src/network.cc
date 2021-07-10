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
#include "wiznet_callbacs.h"

///@{
/// Defined in main.c.
extern CRC_HandleTypeDef hcrc;
extern I2C_HandleTypeDef hi2c2;
///@}

///@{
/// Defined in linker script.
extern std::uint32_t _firmware_updater_start[];
extern std::uint32_t _firmware_updater_pages[];
extern std::uint32_t _main_program_size[];
extern std::uint32_t _flash_end[];
///@}

namespace {
/// Stores main program flash size in words.
const std::uint32_t kMainProgramSize =
    reinterpret_cast<std::uint32_t>(_main_program_size) / 4u;

std::uint16_t buffer_offset;

/**
 * Stores network information.
 * Contains MAC address, Source IP, Subnet mask etc.
 */
wiz_NetInfo net_info{};

inline void UpdateIp() {
  wizchip_getnetinfo(&net_info);
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);

  std::uint8_t level_number = 18u - net_info.ip[2];
  std::uint8_t room_number = net_info.ip[3] - 5u;
  buffer_offset = (level_number * 96u + room_number * 6u) + 2u;
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

  // DHCP 1s timer located in interrupt.cc
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

template <std::size_t N>
std::tuple<std::int32_t, std::array<std::uint8_t, N>,
           std::array<std::uint8_t, 4u>, std::uint16_t>
Network::CheckIpAddress(const std::uint8_t &socket_number) {
  std::array<std::uint8_t, N> buffer{};
  std::array<std::uint8_t, 4u> server_address{};
  std::uint16_t server_port;
  std::int32_t size{recvfrom(socket_number, buffer.data(), buffer.size(),
                             server_address.data(), &server_port)};

  if (size < 0 || (!std::equal(std::begin(net_info.gw), std::end(net_info.gw),
                               server_address.begin()) &&
                   server_address[2] != 0u)) {
    return std::make_tuple(-1, std::move(buffer), std::move(server_address),
                           std::move(server_port));
  }

  return std::make_tuple(std::move(size), std::move(buffer),
                         std::move(server_address), std::move(server_port));
}

void Network::HandleAnimationProtocol() {
  auto [size, buffer, server_address,
        server_port]{CheckIpAddress<1500u>(kAnimationSocket)};
  // Handle too small and incorrect packages
  if (buffer[0] != kAnimationProtocolVersion ||
      (buffer[0] == kAnimationProtocolVersion &&
       size < kAnimationProtocolSize)) {
    return;
  }

  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_SET);
  Panel::SetInternalAnimation(false);

  auto buffer_begin{buffer.begin() + buffer_offset};
  Panel::PanelColorData colors{};
  auto colors_begin{colors.begin()};

  for (auto [i, bytes] = std::tuple(buffer_begin, 0u);; i++, bytes++) {
    if (bytes == 3u || bytes == 9u) {
      // Jump to next row
      i = i - 3u + 48u;
    } else if (bytes == 6u) {
      Panel::GetPanel(Panel::LEFT).SendPixels(colors);
      i -= 48u;
      colors_begin = colors.begin();
    } else if (bytes == 12u) {
      Panel::GetPanel(Panel::RIGHT).SendPixels(colors);
      break;
    }

    *colors_begin = (*i & 0xF0u) >> 4u;
    colors_begin++;
    *colors_begin = *i & 0x0Fu;
    colors_begin++;
  }

  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_RESET);
}

void Network::HandleCommandProtocol() {
  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_SET);

  auto [size, buffer, server_address,
        server_port]{CheckIpAddress<32u>(kCommandSocket)};
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
    case Command::kDisableLeftPanel:
      Panel::GetPanel(Panel::LEFT).SetStatus(Panel::kDisabled);
      break;
    case Command::kDisableRightPanel:
      Panel::GetPanel(Panel::RIGHT).SetStatus(Panel::kDisabled);
      break;
    case kResetLeftPanel:
      Panel::GetPanel(Panel::LEFT).SetStatus(Panel::kPowerOff);
      break;
    case kResetRightPanel:
      Panel::GetPanel(Panel::RIGHT).SetStatus(Panel::kPowerOff);
      break;
    case Command::kUseExternalAnim:
      Panel::SetInternalAnimation(false);
      break;
    case Command::kUseInternalAnim:
      Panel::SetInternalAnimation(true);
      break;
    case Command::kBlank:
      Panel::BlankAll();
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
      std::array<char, 18u> mac{};
      sendto(kCommandSocket, reinterpret_cast<std::uint8_t *>(mac.data()),
             std::snprintf(mac.data(), mac.size(), "%x:%x:%x:%x:%x:%x",
                           net_info.mac[0], net_info.mac[1], net_info.mac[2],
                           net_info.mac[3], net_info.mac[4], net_info.mac[5]),
             server_address.data(), server_port);
      break;
    }
    case Command::kFlushSocketBuffers:
      FlushSocketBuffers();
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
      Panel::WhiteBalanceData white_balance{};
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

      if (socket(kFirmwareUpdaterSocket, Sn_MR_TCP, kFirmwareUpdaterPort,
                 0x00u) != kFirmwareUpdaterSocket) {
        break;
      }

      if (listen(kFirmwareUpdaterSocket) != SOCK_OK) {
        break;
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
        disconnect(kFirmwareUpdaterSocket);
        close(kFirmwareUpdaterSocket);
        break;
      }

      // Write flash page by page
      std::int32_t recv_size{0u};
      std::uint32_t base_addr{
          reinterpret_cast<std::uint32_t>(_firmware_updater_start)};
      do {
        std::array<std::uint8_t, FLASH_PAGE_SIZE> buffer{};
        std::uint32_t *buffer_p{
            reinterpret_cast<std::uint32_t *>(buffer.data())};
        recv_size =
            recv(kFirmwareUpdaterSocket, buffer.data(), FLASH_PAGE_SIZE);

        // Overwrite protection
        if (base_addr + recv_size >=
            reinterpret_cast<std::uint32_t>(_flash_end)) {
          disconnect(kFirmwareUpdaterSocket);
          close(kFirmwareUpdaterSocket);
          break;
        }

        if (recv_size > 0) {
          for (std::size_t i{0u}; i < recv_size / 4u; i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_addr + i * 4u,
                                  buffer_p[i]) != HAL_OK) {
              // If this fails we can't do much
              disconnect(kFirmwareUpdaterSocket);
              close(kFirmwareUpdaterSocket);
              break;
            }
          }

          // Handle odd recv_size, write last byte
          // This should not be called because of alignment
          if (recv_size % 2u != 0u) {
            std::int32_t last_byte = recv_size - 1u;
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_addr + last_byte,
                                  buffer[last_byte]) != HAL_OK) {
              // If this fails we can't do much
              disconnect(kFirmwareUpdaterSocket);
              close(kFirmwareUpdaterSocket);
              break;
            }
          }

          base_addr += recv_size;
        } else {
          disconnect(kFirmwareUpdaterSocket);
          close(kFirmwareUpdaterSocket);
          break;
        }

        getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
      } while (getSn_RX_RSR(kFirmwareUpdaterSocket) != 0u ||
               status != SOCK_CLOSE_WAIT);

      send(kFirmwareUpdaterSocket, (std::uint8_t *)"FLASH_OK", 8u);
      disconnect(kFirmwareUpdaterSocket);
      close(kFirmwareUpdaterSocket);

      // The function HAL_FLASH_Lock() should be called after to lock the FLASH
      // interface
      // Should not fail
      HAL_FLASH_Lock();
      break;
    }
  }

  HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_RESET);
}

void Network::FlushSocketBuffers() {
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
