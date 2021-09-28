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
  static constexpr std::uint8_t kFirmwareUpdaterSocket{3u};

  Network(const Network &) = delete;
  Network &operator=(const Network &) = delete;

  /// Returns a Network instance.
  static Network &Instance();

  /// Network class' loop.
  void Step();

  void UpdateIp();

  void IpAssign();

  void IpUpdate();

  static void IpConflict();

 private:
  static constexpr std::uint16_t kMtu{1500u};

  /// Command socket port number.
  static constexpr std::uint16_t kCommandSocketPort{50000u};

  /// Animation socket port number.
  static constexpr std::uint16_t kAnimationSocketPort{50001u};

  /// Expected animation protocol packet size.
  static constexpr std::uint16_t kAnimationProtocolSize{1250u};

  /// EUI-48 MAC start address.
  static constexpr std::uint16_t kEui48MacStartAddress{0xFAu};

  /// EEPROM I2C device address.
  static constexpr std::uint8_t kEepromAddress{0b10100001u};

  /// DHCP socket number.
  static constexpr std::uint8_t kDhcpSocket{0u};

  /// Command socket number.
  static constexpr std::uint8_t kCommandSocket{1u};

  /// Animation socket number.
  static constexpr std::uint8_t kAnimationSocket{2u};

  /// Currently supported animation protocol version number.
  static constexpr std::uint8_t kAnimationProtocolVersion{2u};

  static constexpr std::uint8_t kCommandProtocolMaxSize{57u};

  Network();

  template <std::size_t N>
  std::tuple<std::int32_t, std::array<std::uint8_t, N>,
             std::array<std::uint8_t, 4u>, std::uint16_t>
  CheckIpAddress(const std::uint8_t &socket_number);

  /// Handles animation protocol.
  void HandleAnimationProtocol();

  /**
   * Handles remote command.
   * @see #Command.
   */
  void HandleCommandProtocol();

  void FlashFirmwareUpdater();

  /**
   * DHCP RX buffer.
   * @note 1 KB should be enough for DHCP RX buffer.
   */
  std::array<std::uint8_t, 1024u> dhcp_rx_buffer_{};

  static constexpr std::array<std::uint8_t, 6> kMulticastHardwareAddress_{
      0x01u, 0x00u, 0x5eu, 0x06u, 0x00u, 0x01u};
  static constexpr std::array<std::uint8_t, 4> kMulticastAddress_{239u, 6u, 0u,
                                                                  1u};

  std::uint16_t firmware_updater_size_{0u};

  std::uint16_t animation_buffer_offset_{0u};
};

#endif  // MATRIX4_MUEB_FW_INC_NETWORK_H_
