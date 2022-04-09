/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#include "network.h"

#include <dhcp.h>
#include <socket.h>

#include <algorithm>
#include <array>
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
    if (dhcp_status == DHCP_FAILED || dhcp_status == DHCP_STOPPED) {
      HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    } else if (dhcp_status == DHCP_IP_ASSIGN ||
               dhcp_status == DHCP_IP_CHANGED ||
               dhcp_status == DHCP_IP_LEASED) {
      HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);

      if (getSn_RX_RSR(kCommandSocket) > 0u) {
        HandleCommandProtocol();
      }

      if (net_info.ip[2] != 0u && net_info.ip[3] != 0u) {
        HAL_NVIC_DisableIRQ(TIM14_IRQn);
        HAL_NVIC_DisableIRQ(TIM16_IRQn);
        HAL_NVIC_DisableIRQ(TIM17_IRQn);

        if (getSn_RX_RSR(kE131SyncSocket) > 0u) {
          HandleE131Packet(kE131SyncSocket);
        }

        if (getSn_RX_RSR(kE131Socket) > 0u) {
          HandleE131Packet(kE131Socket);
        }

        if (getSn_RX_RSR(kArtNetSocket) > 0u) {
          HandleArtNetPacket(kArtNetSocket);
        }

        HAL_NVIC_EnableIRQ(TIM17_IRQn);
        HAL_NVIC_EnableIRQ(TIM16_IRQn);
        HAL_NVIC_EnableIRQ(TIM14_IRQn);
      }
    }
  } else {
    HAL_GPIO_WritePin(LED_JOKER_GPIO_Port, LED_JOKER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
    DhcpRebind();
  }
}

void Network::OpenMulticastSocket(const std::uint8_t &socket_number,
                                  const std::uint8_t &third_octet,
                                  const std::uint8_t &last_octet) {
  std::array<std::uint8_t, 4> multicast_address{239u, 255u, third_octet,
                                                last_octet};
  std::array<std::uint8_t, 6> multicast_hardware_address{
      0x01u, 0x00u, 0x5eu, 0x7fu, third_octet, last_octet};

  // Configure multicast, IGMPv2
  setSn_DHAR(socket_number,
             const_cast<std::uint8_t *>(multicast_hardware_address.data()));
  setSn_DIPR(socket_number,
             const_cast<std::uint8_t *>(multicast_address.data()));
  setSn_DPORT(socket_number, ACN_SDT_MULTICAST_PORT);

  socket(socket_number, Sn_MR_UDP, ACN_SDT_MULTICAST_PORT,
         SF_MULTI_ENABLE | SF_BROAD_BLOCK);
}

void Network::UpdateIp() {
  wizchip_getnetinfo(&net_info);
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_SET);

  std::uint8_t level_number = 18u - net_info.ip[2];
  std::uint8_t room_number = net_info.ip[3] - 5u;

  dmx_buffer_offset_ = (level_number * 192u + room_number * 12u) % 384u;

  universe_number_ = ((level_number * 8u + room_number) / 16u) + 1u;
  OpenMulticastSocket(kE131Socket, 0u, universe_number_);

  art_poll_reply_.ip_address = std::to_array(net_info.ip);
  art_poll_reply_.bind_ip = art_poll_reply_.ip_address;
  boost::endian::store_little_u32(
      broadcast_ip_address_.data(),
      (*(std::uint32_t *)net_info.ip & *(std::uint32_t *)net_info.sn) |
          ~*(std::uint32_t *)net_info.sn);

  art_poll_reply_.sw_in[0] = universe_number_;

  sendto(kArtNetSocket, (std::uint8_t *)&art_poll_reply_, sizeof(ArtPollReply),
         (std::uint8_t *)broadcast_ip_address_.data(), kArtNetPort);
}

void Network::IpAssign() {
  default_ip_assign();
  UpdateIp();
}

void Network::IpUpdate() {
  default_ip_update();
  UpdateIp();
}

void Network::SyncTimedOut() { synced_ = false; }

void Network::IpConflict() {
  default_ip_conflict();
  HAL_GPIO_WritePin(LED_DHCP_GPIO_Port, LED_DHCP_Pin, GPIO_PIN_RESET);
}

void Network::StreamTerminated() {
  HAL_GPIO_WritePin(LED_SERVER_GPIO_Port, LED_SERVER_Pin, GPIO_PIN_RESET);

  if (synchronization_address_ != 0u && !force_synchronization_) {
    return;
  }

  cid_.fill(0u);
  last_ip_address_.fill(0u);
  synchronization_address_ = 0;
  data_sequence_number_ = 0;
  sync_sequence_number_ = 0;
  priority_ = 0;
  synced_ = false;
  force_synchronization_ = false;

  HAL_TIM_Base_Stop_IT(&htim16);
  HAL_NVIC_ClearPendingIRQ(TIM16_IRQn);
  htim16.Instance->EGR = TIM_EGR_UG;
  CLEAR_BIT(htim16.Instance->SR, TIM_SR_UIF);

  close(kE131SyncSocket);
  HAL_TIM_Base_Stop_IT(&htim17);
  HAL_NVIC_ClearPendingIRQ(TIM17_IRQn);
  htim17.Instance->EGR = TIM_EGR_UG;
  CLEAR_BIT(htim17.Instance->SR, TIM_SR_UIF);

  Panel::BlankAll();
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

  // DHCP, command protocol, firmware socket, E1.31 data/sync, E1.31 sync,
  // Art-Net rx/tx sizes
  std::array<std::uint8_t, 8u> txsize{1u, 1u, 1u, 1u, 1u, 1u, 0u, 0u};
  std::array<std::uint8_t, 8u> rxsize{1u, 1u, 4u, 2u, 2u, 2u, 0u, 0u};

  // This includes soft reset
  wizchip_init(txsize.data(), rxsize.data());

  // Gets MAC address from EEPROM.
  // The device uses Microchip 24AA02E48T-I/OT EEPROM
  HAL_I2C_Mem_Read(&hi2c2, kEepromAddress, kEui48MacStartAddress,
                   I2C_MEMADD_SIZE_8BIT, net_info.mac, 6u, HAL_MAX_DELAY);
  setSHAR(net_info.mac);
  art_poll_reply_.mac = std::to_array(net_info.mac);

  // Set all capable, Auto-negotiation enabled
  wiz_PhyConf_t phyconf{PHY_CONFBY_SW, PHY_MODE_AUTONEGO};
  wizphy_setphyconf(&phyconf);

  // DHCP 1s timer located in interrupts.cc
  DHCP_init(kDhcpSocket, dhcp_rx_buffer_.data());

  socket(kCommandSocket, Sn_MR_UDP, kCommandSocketPort, 0x00u);
  socket(kArtNetSocket, Sn_MR_UDP, kArtNetPort, 0x00u);
}

auto Network::CheckIpAddress(const std::uint8_t &socket_number) {
  struct _ {
    std::int32_t size{0u};
    std::array<std::uint8_t, 1472u> buffer{};
    std::array<std::uint8_t, 4u> server_address{};
    std::uint16_t server_port{0u};
  } ret;

  ret.size = recvfrom(socket_number, ret.buffer.data(),
                      static_cast<std::uint16_t>(ret.buffer.size()),
                      ret.server_address.data(), &ret.server_port);
  if (ret.size < 0 ||
      (!std::equal(std::begin(net_info.gw), std::end(net_info.gw),
                   ret.server_address.begin()) &&
       ret.server_address[2] != 0u)) {
    return ret;
  }

  return ret;
}

void Network::SetPanelColorData(const std::uint8_t *data) {
  HAL_GPIO_WritePin(LED_SERVER_GPIO_Port, LED_SERVER_Pin, GPIO_PIN_SET);
  Panel::SetInternalAnimation(false);

  Panel::ColorData colors{};
  auto colors_begin{colors.begin()};
  auto data_begin{data + dmx_buffer_offset_};

  for (auto [i, bytes] = std::tuple(data_begin, 0u);; i++, bytes++) {
    if (bytes == 6u || bytes == 18u) {
      // Jump to next row
      i = i - 6u + 96u;
    }

    if (bytes == 12u) {
      // Set left panel data
      Panel::GetPanel(Panel::Side::LEFT).SetColorData(colors);

      i -= 96u;

      colors_begin = colors.begin();
    } else if (bytes == 24u) {
      // Set right panel data
      Panel::GetPanel(Panel::Side::RIGHT).SetColorData(colors);
      break;
    }

    *colors_begin = (*i * 15u + 135u) >> 8u;
    colors_begin++;
  }
}

void Network::HandleE131Packet(const std::uint8_t &socket_number) {
  const auto [size, buffer, server_address,
              server_port]{CheckIpAddress(socket_number)};
  if (size <= 0) {
    return;
  }

  const auto root_layer = (RootLayer *)buffer.data();
  if (root_layer->preamble_size != 0x0010u ||
      root_layer->post_amble_size != 0x0000u ||
      root_layer->acn_packet_identifier != kAcnPacketIdentifier) {
    return;
  }

  if (root_layer->vector == VECTOR_ROOT_E131_DATA) {
    const auto e131DataPacket = (E131DataPacket *)buffer.data();

    const bool stream_terminated =
        e131DataPacket->framing_layer.options & 0x40u;
    const bool force_synchronization =
        e131DataPacket->framing_layer.options & 0x20u;
    const auto sequence_number =
        e131DataPacket->framing_layer.sequence_number - data_sequence_number_;
    if (e131DataPacket->framing_layer.vector != VECTOR_E131_DATA_PACKET ||
        (data_sequence_number_ > 0u && sequence_number <= 0 &&
         sequence_number > -20) ||
        e131DataPacket->framing_layer.universe == 0u ||
        e131DataPacket->framing_layer.universe > 63999u ||
        e131DataPacket->framing_layer.priority > 200u ||
        e131DataPacket->framing_layer.priority < priority_ ||
        (data_sequence_number_ > 0u && e131DataPacket->root_layer.cid != cid_ &&
         e131DataPacket->framing_layer.priority == priority_) ||
        e131DataPacket->framing_layer.options & 0x80u || stream_terminated ||
        (synchronization_address_ != 0u &&
         (!force_synchronization && !synced_)) ||
        e131DataPacket->dmp_layer.vector != VECTOR_DMP_SET_PROPERTY ||
        e131DataPacket->dmp_layer.address_type_and_data_type != 0xa1u ||
        e131DataPacket->dmp_layer.first_property_address != 0x0000u ||
        e131DataPacket->dmp_layer.address_increment != 0x0001u ||
        e131DataPacket->dmp_layer.property_value_count > 513u ||
        e131DataPacket->dmp_layer.property_value_count < 384u) {
      if (stream_terminated) {
        StreamTerminated();
      }

      return;
    }
    cid_ = e131DataPacket->root_layer.cid;
    priority_ = e131DataPacket->framing_layer.priority;
    force_synchronization_ = force_synchronization;
    data_sequence_number_ = e131DataPacket->framing_layer.sequence_number;

    if (synchronization_address_ != 0u &&
        e131DataPacket->framing_layer.synchronization_address != 0u &&
        synchronization_address_ !=
            e131DataPacket->framing_layer.synchronization_address) {
      return;
    }

    if (synchronization_address_ == 0u &&
        e131DataPacket->framing_layer.synchronization_address != 0u) {
      const auto lowByte = static_cast<std::uint8_t>(
          e131DataPacket->framing_layer.synchronization_address & 0x00FFu);
      const auto highByte = static_cast<std::uint8_t>(
          (e131DataPacket->framing_layer.synchronization_address & 0xFF00u) >>
          8u);

      if (highByte != 0u || lowByte != universe_number_) {
        OpenMulticastSocket(kE131SyncSocket, highByte, lowByte);
      }

      synced_ = false;
    } else if (synchronization_address_ != 0u &&
               e131DataPacket->framing_layer.synchronization_address == 0u) {
      synced_ = false;
      close(kE131SyncSocket);
    }
    synchronization_address_ =
        e131DataPacket->framing_layer.synchronization_address;

    SetPanelColorData(e131DataPacket->dmp_layer.property_values.data() + 1);

    if (synchronization_address_ == 0u) {
      Panel::SendColorDataAll();
    }

    HAL_TIM_Base_Stop_IT(&htim16);
    HAL_NVIC_ClearPendingIRQ(TIM16_IRQn);
    htim16.Instance->EGR = TIM_EGR_UG;
    CLEAR_BIT(htim16.Instance->SR, TIM_SR_UIF);
    HAL_TIM_Base_Start_IT(&htim16);
  } else if (root_layer->vector == VECTOR_ROOT_E131_EXTENDED) {
    const auto e131SyncPacket = (E131SyncPacket *)buffer.data();

    const auto sequence_number =
        e131SyncPacket->framing_layer.sequence_number - sync_sequence_number_;
    if (e131SyncPacket->framing_layer.vector !=
            VECTOR_E131_EXTENDED_SYNCHRONIZATION ||
        (sync_sequence_number_ > 0u && sequence_number <= 0 &&
         sequence_number > -20) ||
        e131SyncPacket->framing_layer.synchronization_address == 0u ||
        e131SyncPacket->framing_layer.synchronization_address !=
            synchronization_address_) {
      return;
    }
    sync_sequence_number_ = e131SyncPacket->framing_layer.sequence_number;

    Panel::SendColorDataAll();
    synced_ = true;

    HAL_TIM_Base_Stop_IT(&htim17);
    HAL_NVIC_ClearPendingIRQ(TIM17_IRQn);
    htim17.Instance->EGR = TIM_EGR_UG;
    CLEAR_BIT(htim17.Instance->SR, TIM_SR_UIF);
    HAL_TIM_Base_Start_IT(&htim17);
  }
}

void Network::HandleArtNetPacket(const std::uint8_t &socket_number) {
  const auto [size, buffer, server_address,
              server_port]{CheckIpAddress(socket_number)};
  if (size <= 0) {
    return;
  }

  const auto packet = (ArtNetHeader *)buffer.data();

  if (packet->art_id_op_code.id != kArtNetId || packet->prot_ver != 14u) {
    return;
  }

  switch (packet->art_id_op_code.op_code) {
    case kOpPoll: {
      sendto(kArtNetSocket, (std::uint8_t *)&art_poll_reply_,
             sizeof(ArtPollReply), (std::uint8_t *)broadcast_ip_address_.data(),
             kArtNetPort);
      break;
    }
    case kOpOutput: {
      const auto art_dmx = (ArtDmx *)buffer.data();
      if (art_dmx->sequence != 0u) {
        const auto sequence_number = art_dmx->sequence - data_sequence_number_;
        if (sequence_number <= 0 && sequence_number > -20) {
          return;
        }

        data_sequence_number_ = art_dmx->sequence;
      }

      if (art_dmx->net != art_poll_reply_.net_switch ||
          art_dmx->sub_uni !=
              (art_poll_reply_.sub_switch << 4u | art_poll_reply_.sw_in[0]) ||
          art_dmx->length < 2u || art_dmx->length > 512u ||
          art_dmx->length < 384u ||
          (last_ip_address_[0] != 0u && last_ip_address_ != server_address)) {
        return;
      }
      last_ip_address_ = server_address;

      SetPanelColorData(art_dmx->data.data());

      if (!synced_) {
        Panel::SendColorDataAll();
      }

      HAL_TIM_Base_Stop_IT(&htim16);
      HAL_NVIC_ClearPendingIRQ(TIM16_IRQn);
      htim16.Instance->EGR = TIM_EGR_UG;
      CLEAR_BIT(htim16.Instance->SR, TIM_SR_UIF);
      HAL_TIM_Base_Start_IT(&htim16);
      break;
    }
    case kOpSync: {
      if (last_ip_address_ != server_address) {
        return;
      }

      synced_ = true;
      Panel::SendColorDataAll();

      HAL_TIM_Base_Stop_IT(&htim14);
      HAL_NVIC_ClearPendingIRQ(TIM14_IRQn);
      htim14.Instance->EGR = TIM_EGR_UG;
      CLEAR_BIT(htim14.Instance->SR, TIM_SR_UIF);
      HAL_TIM_Base_Start_IT(&htim14);
      break;
    }
  }
}

void Network::HandleCommandProtocol() {
  auto [size, buffer, server_address,
        server_port]{CheckIpAddress(kCommandSocket)};
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
