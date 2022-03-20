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
#include <tuple>

#include "crc.h"
#include "e131.h"
#include "gpio.h"
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

/**
 * Stores network information.
 * Contains MAC address, Source IP, Subnet mask etc.
 */
wiz_NetInfo net_info{};

void CIpAssign() { Network::Instance().IpAssign(); }

void CIpUpdate() { Network::Instance().IpUpdate(); }

void CIpConflict() { Network::IpConflict(); }

}  // namespace

Network &Network::Instance() {
  static Network network;

  return network;
}

void Network::Step() {
  if (wizphy_getphylink() == PHY_LINK_ON) {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_SET);

    uint8_t dhcp_status = DHCP_run();
    if (dhcp_status == DHCP_FAILED || dhcp_status == DHCP_RUNNING) {
      HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    } else if (dhcp_status == DHCP_IP_ASSIGN ||
               dhcp_status == DHCP_IP_CHANGED ||
               dhcp_status == DHCP_IP_LEASED) {
      HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);
    }

    if (getSn_RX_RSR(kCommandSocket) > 0u) {
      HandleCommandProtocol();
    }

    if (getSn_RX_RSR(kE131Socket) > 0u && net_info.ip[2] != 0u &&
        net_info.ip[3] != 0u) {
      HandleE131Protocol();
    }
  } else {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    DhcpRebind();
  }
}

void Network::UpdateIp() {
  wizchip_getnetinfo(&net_info);
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);

  std::uint8_t level_number = 18u - net_info.ip[2];
  std::uint8_t room_number = net_info.ip[3] - 5u;
  std::uint8_t multicast_number = ((level_number * 8u + room_number) / 16u) + 1;

  std::array<std::uint8_t, 4> multicast_address{239u, 255u, 0u,
                                                multicast_number};
  std::array<std::uint8_t, 6> multicast_hardware_address{
      0x01u, 0x00u, 0x5eu, 0x7fu, 0x00u, multicast_number};

  // Configure multicast, IGMPv2
  setSn_DHAR(kE131Socket,
             const_cast<std::uint8_t *>(multicast_hardware_address.data()));
  setSn_DIPR(kE131Socket, const_cast<std::uint8_t *>(multicast_address.data()));
  setSn_DPORT(kE131Socket, ACN_SDT_MULTICAST_PORT);

  animation_buffer_offset_ = static_cast<std::uint16_t>(
      ((level_number * 192u + room_number * 12u) % 384u) + 1u);
}

void Network::IpAssign() {
  default_ip_assign();
  UpdateIp();

  socket(kE131Socket, Sn_MR_UDP, ACN_SDT_MULTICAST_PORT, SF_MULTI_ENABLE);
}

void Network::IpUpdate() {
  default_ip_update();
  UpdateIp();

  socket(kE131Socket, Sn_MR_UDP, ACN_SDT_MULTICAST_PORT, SF_MULTI_ENABLE);
}

void Network::IpConflict() {
  default_ip_conflict();
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
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
  reg_dhcp_cbfunc(CIpAssign, CIpUpdate, CIpConflict);

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

void Network::HandleE131Protocol() {
  auto [size, buffer, server_address,
        server_port]{CheckIpAddress<kE131ProtocolMaxSize>(kE131Socket)};
  HAL_GPIO_WritePin(LED_SERVER_GPIO_Port, LED_SERVER_Pin, GPIO_PIN_SET);
  Panel::SetInternalAnimation(false);

  auto e131Packet = (E131Packet *)buffer.data();
  if (e131Packet->root_layer.preamble_size != 0x0010u ||
      e131Packet->root_layer.post_amble_size != 0 ||
      e131Packet->root_layer.acn_packet_identifier != kAcn_packet_identifier ||
      (e131Packet->root_layer.vector != VECTOR_ROOT_E131_DATA &&
       e131Packet->root_layer.vector != VECTOR_ROOT_E131_EXTENDED) ||
      e131Packet->framing_layer.priority > 200u ||
      (e131Packet->framing_layer.options & 0x80u) == 0x80u ||
      (e131Packet->framing_layer.options & 0x40u) == 0x40u ||
      e131Packet->framing_layer.universe == 0u ||
      e131Packet->framing_layer.universe > 63999u ||
      e131Packet->framing_layer.vector != VECTOR_E131_DATA_PACKET ||
      e131Packet->framing_layer.vector != VECTOR_E131_EXTENDED_DISCOVERY) {
    return;
  }

  if (e131Packet->root_layer.vector == VECTOR_ROOT_E131_DATA) {
    if (e131Packet->framing_layer.vector == VECTOR_E131_DATA_PACKET) {
      if (e131Packet->dmp_layer.vector != VECTOR_DMP_SET_PROPERTY ||
          e131Packet->dmp_layer.address_type_and_data_type != 0xa1u ||
          e131Packet->dmp_layer.first_property_address != 0x0000u ||
          e131Packet->dmp_layer.address_increment != 0x0001u ||
          e131Packet->dmp_layer.property_value_count > 513u ||
          e131Packet->dmp_layer.property_value_count < 385u) {
        return;
      }

      auto property_begin{e131Packet->dmp_layer.property_values.data() +
                          animation_buffer_offset_};
      Panel::ColorData colors{};
      auto colors_begin{colors.begin()};

      for (auto [i, bytes] = std::tuple(property_begin, 0u);; i++, bytes++) {
        if (bytes == 6u || bytes == 18u) {
          // Jump to next row
          i = i - 6u + 96u;
        } else if (bytes == 12u) {
          // Send left panel data
          Panel::GetPanel(Panel::Side::LEFT).SendColorData(colors);
          i -= 96u;
          colors_begin = colors.begin();
        } else if (bytes == 24u) {
          // Send right panel data
          Panel::GetPanel(Panel::Side::RIGHT).SendColorData(colors);
          break;
        }

        *colors_begin = (*i * 15u + 135u) >> 8u;
        colors_begin++;
      }
    }
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

  HAL_GPIO_WritePin(LED_SERVER_GPIO_Port, LED_SERVER_Pin, GPIO_PIN_SET);

  switch (static_cast<Command>(buffer[3])) {
      // Mutable commands
    case Command::kDisablePanels:
      Panel::DisableAll();
      break;
    case Command::kSetPanelsWhiteBalance: {
      Panel::WhiteBalanceData white_balance{};
      std::copy_n(buffer.begin() + 11u, white_balance.size(),
                  white_balance.begin());
      Panel::SendWhiteBalanceToAll(white_balance);
      break;
    }
    case Command::kSetPanelWhiteBalance: {
      Panel::WhiteBalanceData white_balance{};
      std::copy_n(buffer.begin() + 12u, white_balance.size(),
                  white_balance.begin());
      Panel::GetPanel(static_cast<Panel::Side>(buffer[11]))
          .SendWhiteBalance(white_balance);
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
    case Command::kBlankPanels:
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
        sendto(kCommandSocket,
               reinterpret_cast<std::uint8_t *>(&firmware_updater_size_), 1u,
               server_address.data(), server_port);
        return;
      }

      // firmware updater size in words
      const std::uint16_t firmware_updater_size{
          static_cast<std::uint16_t>(firmware_updater_size_ / 4u)};
      auto crc{__REV(~HAL_CRC_Calculate(
          &hcrc, reinterpret_cast<std::uint32_t *>(firmware_updater_start),
          firmware_updater_size))};

      sendto(kCommandSocket, (std::uint8_t *)&crc, sizeof(crc),
             server_address.data(), server_port);
      break;
    }
    case Command::kGetPanelStates:
      sendto(kCommandSocket, Panel::GetPanelStates().data(), 2,
             server_address.data(), server_port);
      break;
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

  // Enable TCP keep alive
  setSn_KPALVTR(kFirmwareUpdaterSocket, 1u);

  if (listen(kFirmwareUpdaterSocket) != SOCK_OK) {
    return;
  }

  std::uint8_t status;
  while (true) {
    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
    if (status == SOCK_ESTABLISHED) {
      break;
    } else if (status == SOCK_CLOSED) {
      close(kFirmwareUpdaterSocket);
      return;
    }
  }

  /* The function HAL_FLASH_Unlock() should be called before to unlock the FLASH
   * interface
   */
  if (HAL_FLASH_Unlock() != HAL_OK) {
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
    close(kFirmwareUpdaterSocket);
    return;
  }

  // Write flash page by page
  std::int32_t received_size;
  std::uint32_t base_address{
      reinterpret_cast<std::uint32_t>(firmware_updater_start)};
  do {
    // Send dummy packet to generate keep alive
    send(Network::kFirmwareUpdaterSocket, (std::uint8_t *)"!", 1u);

    if (getSn_RX_RSR(kFirmwareUpdaterSocket) == 0u) {
      if (status == SOCK_CLOSE_WAIT) {
        break;
      }

      continue;
    }

    std::array<std::uint8_t, FLASH_PAGE_SIZE> flash_page_buffer{};
    std::uint32_t *flash_page_buffer_p{
        reinterpret_cast<std::uint32_t *>(flash_page_buffer.data())};

    received_size =
        recv(kFirmwareUpdaterSocket, flash_page_buffer.data(), FLASH_PAGE_SIZE);
    if (received_size < 0) {
      return;
    }

    // Overwrite protection
    if (base_address + received_size >=
        reinterpret_cast<std::uint32_t>(flash_end)) {
      close(kFirmwareUpdaterSocket);
      return;
    }

    std::size_t iterations = received_size / 4u;
    for (std::size_t i{0u}; i < iterations || (iterations == 0 && i < 1); i++) {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_address + i * 4u,
                            flash_page_buffer_p[i]) != HAL_OK) {
        close(kFirmwareUpdaterSocket);
        return;
      }
    }

    base_address += received_size;
    firmware_updater_size_ += received_size;
    getsockopt(kFirmwareUpdaterSocket, SO_STATUS, &status);
  } while (getSn_RX_RSR(Network::kFirmwareUpdaterSocket) != 0u ||
           status != SOCK_CLOSE_WAIT);

  send(kFirmwareUpdaterSocket, (std::uint8_t *)"FLASH_OK", 8u);
  disconnect(kFirmwareUpdaterSocket);
  close(kFirmwareUpdaterSocket);

  /* The function HAL_FLASH_Lock() should be called after to lock the FLASH
   * interface
   */
  HAL_FLASH_Lock();
}
