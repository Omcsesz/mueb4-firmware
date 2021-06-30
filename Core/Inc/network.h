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

  static Network &Instance();

  /// Network class' loop.
  void Step();

 private:
  static constexpr std::array<std::uint8_t, 4u>
      kAnimationSocketMulticastAddress{239, 6, 0, 1};

  /// Command socket port number.
  static constexpr std::uint16_t kCommandSocketPort{50000u};

  /// Animation socket port number.
  static constexpr std::uint16_t kAnimationSocketPort{50001u};

  /// Firmware updater flasher port number.
  static constexpr std::uint16_t kFirmwareUpdaterFlasherPort{50002u};

  /// EUI-48 value's start address.
  static constexpr std::uint16_t kEui48StartAddress{0xFAu};

  /// EEPROM I2C device address.
  static constexpr std::uint8_t kEepromAddress{0b10100001u};

  /// Socket number for DHCP.
  static constexpr std::uint8_t kDhcpSocket{0u};

  /// Socket number for remote command handling.
  static constexpr std::uint8_t kCommandSocket{1u};

  /// Socket number for animation protocol.
  static constexpr std::uint8_t kAnimationSocket{2u};

  /// Socket number for firmware updater flasher.
  static constexpr std::uint8_t kFirmwareUpdaterFlasherSocket{3u};

  Network();

  template <std::size_t N>
  std::tuple<std::int32_t, std::array<std::uint8_t, N>,
             std::array<std::uint8_t, 4u>, std::uint16_t>
  HandlePacket(const std::uint8_t &socket_number);

  /// Handles animation protocol.
  void HandleAnimationProtocol();

  /// Handles remote command @see #Command.
  void HandleCommandProtocol();

  /// Flushes socket buffers.
  void FlushBuffers();

  /**
   * DHCP RX buffer.
   * @note 1 kb should be enough for DHCP RX buffer.
   */
  std::array<std::uint8_t, 1024u> dhcp_rx_buffer_{};
};

#endif  // MATRIX4_MUEB_FW_INC_NETWORK_H_
