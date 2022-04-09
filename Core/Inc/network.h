/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#ifndef MATRIX4_MUEB_FW_INC_NETWORK_H_
#define MATRIX4_MUEB_FW_INC_NETWORK_H_

#include <array>
#include <cstdint>
#include <tuple>

#include "art_net.h"

/**
 * Manages all network related functionality.
 */
class Network final {
 public:
  /// Byte code for network commands.
  enum class Command {
    // Mutable commands
    kDisablePanels = 0x00u,          ///< Disable all panels
    kSetPanelsWhiteBalance = 0x01u,  ///< Set panels white balance
    kSetPanelWhiteBalance = 0x02u,   ///< Set one panel white balance
    kUseInternalAnimation = 0x03u,   ///< Use internal animation
    kUseExternalAnimation = 0x04u,   ///< Use external animation
    kSwapPanels = 0x05u,             ///< Swap left and right panels
    kBlankPanels = 0x06u,            ///< Blank both panels
    kReset = 0x07u,                  ///< Reboot device
    kStartFirmwareUpdate = 0x08u,    ///< Start firmware update process
    kFlashFirmwareUpdater = 0x09u,   ///< Flash firmware updater
    // Immutable commands
    kPing = 0x0Au,                 ///< Send back 'pong' response
    kGetStatus = 0x0Bu,            ///< Get device's status
    kGetMac = 0x0Cu,               ///< Get device's MAC address
    kGetFirmwareChecksum = 0x0Du,  ///< Return main program checksum
    kGetFirmwareUpdaterChecksum =
        0x0Eu,  ///< Return firmware updater checksum, can be used after
                ///< kFlashFirmwareUpdater
    kGetPanelStates = 0x0Fu
  };

  /// Firmware updater port number.
  static constexpr std::uint16_t kFirmwareUpdaterPort{50002u};

  /// Firmware updater Socket number.
  static constexpr std::uint8_t kFirmwareUpdaterSocket{2u};

  Network(const Network &) = delete;
  Network &operator=(const Network &) = delete;

  /// Returns a Network instance.
  static Network &Instance();

  /// Network class' loop.
  void Step();

  void UpdateIp();

  void IpAssign();

  void IpUpdate();

  void SyncTimedOut();

  static void IpConflict();

  void StreamTerminated();

 private:
  /// Command socket port number.
  static constexpr std::uint16_t kCommandSocketPort{50000u};

  /// EUI-48 MAC start address.
  static constexpr std::uint16_t kEui48MacStartAddress{0xFAu};

  /// EEPROM I2C device address.
  static constexpr std::uint8_t kEepromAddress{0b10100001u};

  /// DHCP socket number.
  static constexpr std::uint8_t kDhcpSocket{0u};

  /// Command socket number.
  static constexpr std::uint8_t kCommandSocket{1u};

  /// Animation socket number.
  static constexpr std::uint8_t kE131Socket{3u};

  static constexpr std::uint8_t kE131SyncSocket{4u};

  static constexpr std::uint8_t kArtNetSocket{5u};

  Network();

  static void OpenMulticastSocket(const std::uint8_t &socket_number,
                                  const std::uint8_t &third_octet,
                                  const std::uint8_t &last_octet);

  static auto CheckIpAddress(const std::uint8_t &socket_number);

  void SetPanelColorData(const std::uint8_t *data);

  /// Handles animation protocol.
  void HandleE131Packet(const std::uint8_t &socket_number);

  void HandleArtNetPacket(const std::uint8_t &socket_number);

  /**
   * Handles remote command.
   * @see #Command.
   */
  void HandleCommandProtocol();

  void FlashFirmwareUpdater();

  /**
   * DHCP RX buffer.
   */
  std::array<std::uint8_t, 576u> dhcp_rx_buffer_{0u};

  ArtPollReply art_poll_reply_;

  std::array<std::uint8_t, 16u> cid_{0u};

  std::array<std::uint8_t, 4u> last_ip_address_{0u};

  std::array<std::uint8_t, 4u> broadcast_ip_address_{0u};

  std::uint16_t firmware_updater_size_{0u};

  std::uint16_t dmx_buffer_offset_{0u};

  std::uint16_t synchronization_address_{0u};

  std::uint8_t data_sequence_number_{0u};

  std::uint8_t sync_sequence_number_{0u};

  std::uint8_t priority_{0u};

  std::uint8_t universe_number_{0u};

  bool synced_{false};

  bool force_synchronization_{false};
};

#endif  // MATRIX4_MUEB_FW_INC_NETWORK_H_
