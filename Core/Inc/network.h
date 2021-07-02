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
  /// Firmware updater port number.
  static constexpr std::uint16_t kFirmwareUpdaterPort{50002u};

  /// Firmware updater Socket number.
  static constexpr std::uint8_t kFirmwareUpdaterSocket{3u};

  /// Byte code for network commands.
  enum Command {
    kTurn12vOffLeft,              ///< Turn left panel's 12v off
    kTurn12vOffRight,             ///< Turn right panel's 12v off
    kResetLeftPanel,              ///< Reset left panel state
    kResetRightPanel,             ///< Reset right panel state
    kReboot,                      ///< Reboot device
    kGetStatus,                   ///< Get device's status
    kGetMac,                      ///< Get device's MAC address
    kUseInternalAnim,             ///< Use internal animation
    kUseExternalAnim,             ///< Use external animation
    kBlank,                       ///< Blank both panels
    kFlushSocketBuffers,          ///< Flush socket buffers
    kPing,                        ///< Send back 'pong' response
    kStartFirmwareUpdate,         ///< Start firmware update process
    kGetFirmwareChecksum,         ///< Return main program checksum
    kGetFirmwareUpdaterChecksum,  ///< Return firmware updater checksum
    kSwapPanels,                  ///< Swap left and right panels
    kSetWhitebalance,             ///< Set white balance
    kFlashFirmwareUpdater         ///< Flash firmware updater
  };

  Network(const Network &) = delete;
  Network &operator=(const Network &) = delete;

  /// Returns a Network instance.
  static Network &Instance();

  /// Network class' loop.
  void Step();

 private:
  /// Animation protocol multicast destination address.
  static constexpr std::array<std::uint8_t, 4u>
      kAnimationSocketMulticastAddress{239u, 6u, 0u, 1u};

  /// Command socket port number.
  static constexpr std::uint16_t kCommandSocketPort{50000u};

  /// Animation socket port number.
  static constexpr std::uint16_t kAnimationSocketPort{50001u};

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

  /// Currently supported protocol version number.
  static constexpr std::uint8_t kAnimationProtocolVersion{2u};

  Network();

  template <std::size_t N>
  std::tuple<std::int32_t, std::array<std::uint8_t, N>,
             std::array<std::uint8_t, 4u>, std::uint16_t>
  HandlePacket(const std::uint8_t &socket_number);

  /// Handles animation protocol.
  void HandleAnimationProtocol();

  /**
   * Handles remote command.
   * @see #Command.
   */
  void HandleCommandProtocol();

  /// Flushes socket buffers.
  void FlushSocketBuffers();

  /**
   * DHCP RX buffer.
   * @note 1 KB should be enough for DHCP RX buffer.
   */
  std::array<std::uint8_t, 1024u> dhcp_rx_buffer_{};
};

#endif  // MATRIX4_MUEB_FW_INC_NETWORK_H_
