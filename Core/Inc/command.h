#ifndef MUEB4_FIRMWARE_CORE_INC_COMMAND_H_
#define MUEB4_FIRMWARE_CORE_INC_COMMAND_H_

#include <array>
#include <boost/endian.hpp>
#include <cstdint>

constexpr std::array<std::uint8_t, 4u> kCommandId{"SEM"};

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
  kSetArtNet = 0x10u,
  kSetE131 = 0x11u,
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

struct CommandHeader {
  const std::array<std::uint8_t, 4u> id;
  const std::uint8_t code;
  const std::uint8_t is_broadcast;
  const std::array<std::uint8_t, 6u> mac{0u};
};

BOOST_STATIC_ASSERT(sizeof(CommandHeader) == 12u);

struct WhiteBalance {
  const CommandHeader header;
  const std::uint8_t side;
  const std::array<std::uint8_t, 45u> data;
};

BOOST_STATIC_ASSERT(sizeof(WhiteBalance) == sizeof(CommandHeader) + 46u);

#endif  // MUEB4_FIRMWARE_CORE_INC_COMMAND_H_
