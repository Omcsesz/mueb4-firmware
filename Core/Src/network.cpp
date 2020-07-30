/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#include "network.hpp"

#include <dhcp.h>
#include <socket.h>

#include <cstdio>

#include "dhcp_buffer.h"
#include "gpios.h"
#include "mac_eeprom.h"
#include "main.h"
#include "version.hpp"
#include "window.hpp"

///@{
/// Defined in linker script
extern std::uint32_t _firmware_update_handler;
extern std::uint32_t _main_program_end;
///@}

extern CRC_HandleTypeDef hcrc;

namespace {
// Calculate main program flash size in words
const std::uint32_t main_program_size =
    ((std::uint32_t)&_main_program_end - FLASH_BASE) / 4;

/**
 * Stores network information.
 * Contains MAC address, Source IP, Subnet mask etc.
 */
wiz_NetInfo netInfo;

/// Stores the level number where the device is located
std::uint8_t level_number{0};
/// Stores the room number where the device is located
std::uint8_t room_number{0};

/// Socket number for remote command handling
static constexpr std::uint8_t command_socket{0};
/// Socket number for unicast protocol
static constexpr std::uint8_t unicast_socket{1};
/// Socket number for broadcast protocol
static constexpr std::uint8_t broadcast_socket{2};
/// Socket number for DHCP
static constexpr std::uint8_t dhcp_socket{7};

/// WIZnet critical enter
void cris_en() { __disable_irq(); }

/// WIZnet critical exit
void cris_ex() { __enable_irq(); }

/// WIZnet chip select
void cs_sel() {
  reset_gpio(SPI1_NSS);  // ChipSelect to low
}

/// WIZnet chip deselect
void cs_desel() {
  set_gpio(SPI1_NSS);  // ChipSelect to high
}

/// Read byte from WIZnet chip through SPI
std::uint8_t spi_rb(void) {
  while (LL_SPI_IsActiveFlag_RXNE(SPI1))
    LL_SPI_ReceiveData8(SPI1);  // flush any FIFO content

  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    ;

  LL_SPI_TransmitData8(SPI1, 0xFF);  // send dummy byte

  while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
    ;

  return (LL_SPI_ReceiveData8(SPI1));
}

/// Write byte to WIZnet chip through SPI
void spi_wb(std::uint8_t b) {
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    ;

  LL_SPI_TransmitData8(SPI1, b);
}

inline void update_ip() {
  wizchip_getnetinfo(&netInfo);
  level_number = netInfo.ip[2];
  room_number = netInfo.ip[3];
}

void ip_assign() {
  default_ip_assign();

  update_ip();
  set_gpio(LED_DHCP);
}

void ip_update() {
  default_ip_update();

  update_ip();
}

void ip_conflict() { reset_gpio(LED_DHCP); }

void flush_buffers() {
  auto size{getSn_RX_RSR(command_socket)};

  if (size) {
    wiz_recv_ignore(command_socket, getSn_RX_RSR(command_socket));
    setSn_CR(command_socket, Sn_CR_RECV);
    while (getSn_CR(command_socket))
      ;
  }

  if ((size = getSn_RX_RSR(unicast_socket))) {
    wiz_recv_ignore(unicast_socket, size);
    setSn_CR(unicast_socket, Sn_CR_RECV);
    while (getSn_CR(unicast_socket))
      ;
  }

  if ((size = getSn_RX_RSR(broadcast_socket))) {
    wiz_recv_ignore(broadcast_socket, size);
    setSn_CR(broadcast_socket, Sn_CR_RECV);
    while (getSn_CR(broadcast_socket))
      ;
  }
}
}  // namespace

/*********************************
 *  network Class function defs
 ********************************/

void network::step_network() {
  if (wizphy_getphylink() == PHY_LINK_ON) {
    set_gpio(LED_JOKER);

    // Can be used without IP address
    if (getSn_RX_RSR(command_socket) > 0) do_remote_command();

    // do DHCP task
    switch (DHCP_run()) {
      case DHCP_IP_ASSIGN:
      case DHCP_IP_CHANGED:
      case DHCP_IP_LEASED:
        // Must have IP address to work
        fetch_frame();
        break;
      case DHCP_FAILED:
        reset_gpio(LED_DHCP);
        break;
      default:
        break;
    }
  } else {
    reset_gpio(LED_JOKER);
    reset_gpio(LED_DHCP);
    DHCP_rebind();
  }
}

network &network::instance() {
  static network net;

  return net;
}

network::network() {
  // Hard-reset W5500
  reset_gpio(W5500_RESET);
  HAL_Delay(1);  // min reset cycle 500 us
  toogle_gpio(W5500_RESET);
  HAL_Delay(1);  // PLL lock 1 ms max (refer datasheet)

  reg_wizchip_cris_cbfunc(cris_en, cris_ex);
  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
  reg_dhcp_cbfunc(ip_assign, ip_update, ip_conflict);

  getMAC(netInfo.mac);
  setSHAR(netInfo.mac);
  netInfo.dhcp = NETINFO_DHCP;

  // Set all capable, Auto-negotiation enabled
  wiz_PhyConf_t phyconf = {.by = PHY_CONFBY_SW, .mode = PHY_MODE_AUTONEGO};
  wizphy_setphyconf(&phyconf);

  // DHCP 1s timer located in stm32f0xx_it.c
  DHCP_init(dhcp_socket, gDATABUF);

  socket(command_socket, Sn_MR_UDP, 2000, 0x00);
  socket(unicast_socket, Sn_MR_UDP, 3000, 0x00);
  socket(broadcast_socket, Sn_MR_UDP, 10000, 0x00);
}

std::size_t network::create_status_string() {
  int ret;

  ret =
      std::snprintf(status_string, sizeof(status_string),
                    "MUEB FW version: %s\n"
                    "MUEB MAC: %x:%x:%x:%x:%x:%x\n"
                    "anim_source: %#x\n"
                    "telemetry_comm_buff: %#x\n"
                    "frame_ether_buff: %#x\n"
                    "SEM forever\n",
                    mueb_version, netInfo.mac[0], netInfo.mac[1],
                    netInfo.mac[2], netInfo.mac[3], netInfo.mac[4],
                    netInfo.mac[5], window::internal_animation_on,
                    getSn_RX_RSR(command_socket), getSn_RX_RSR(unicast_socket));

  return (ret >= 0) ? ret : 0;
}

void network::fetch_frame_unicast_proto() {
  window::internal_animation_on = false;

  toogle_gpio(LED_COMM);

  std::uint8_t buff[5]{};
  std::uint8_t svr_addr[4];
  std::uint16_t svr_port;

  if (recvfrom(unicast_socket, buff, sizeof(buff), svr_addr, &svr_port) < 5 ||
      buff[1] >= window::num_of_pixels)
    return;

  std::uint8_t pixel_num{buff[1]};
  bool window{buff[0]};
  window::get_window(static_cast<window::window_from_outside>(window))
      .pixels[pixel_num]
      .set(buff[2], buff[3], buff[4]);
}

void network::fetch_frame_broadcast_proto() {
  window::internal_animation_on = false;

  toogle_gpio(LED_COMM);

  const std::uint8_t level{level_number};
  const std::uint8_t room{room_number};
  std::uint8_t buff[1500]{};
  std::uint8_t svr_addr[4];
  std::uint16_t svr_port;

  auto size{
      recvfrom(broadcast_socket, buff, sizeof(buff), svr_addr, &svr_port)};

  if ((buff[0] != 0x01 && buff[0] != 0x02) || (buff[0] == 0x01 && size < 314) ||
      (buff[0] == 0x02 && size < 1250))
    return;

  auto &first_window{window::get_window(window::LEFT)};
  auto &second_window{window::get_window(window::RIGHT)};

  std::uint8_t r{0}, g{0}, b{0};

  std::uint32_t base_offset{0};
  std::size_t running_offset{0};

  if (buff[0] == 0x01) {
    /* Read compressed pixels window wise from frame buffer
     * Data can be read as a 32x26 divided 2D array
     * One frame is divided into 4 packets
     * room row * 16(8 room per row * 2 pixel horizontally)
     * room column * 2 pixel vertically
     * 52 window per packet
     */
    std::uint8_t pn_expected = ((18 - level) * 16 + (room - 5) * 2) / 52;
    std::uint8_t pn{buff[1]};

    if (pn != pn_expected) return;

    /* Calculate frame buffer offset from room index
     * room row * room per row
     * room column
     * 26 room per packet(2 window per room, 52 / 2)
     * 12 byte per window
     * 2 byte packet header
     */
    base_offset = (((18 - level) * 8 + (room - 5)) % 26) * 12 + 2;

    //----------------------------------

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    first_window.pixels[0].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    first_window.pixels[1].set(r, g, b);

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    first_window.pixels[2].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    first_window.pixels[3].set(r, g, b);

    //---------------------------------

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    second_window.pixels[0].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    second_window.pixels[1].set(r, g, b);

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    second_window.pixels[2].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    second_window.pixels[3].set(r, g, b);
  } else if (buff[0] == 0x02) {
    /* Read compressed pixels row wise from frame buffer using 2x2 window
     * 2x2 window position is based on room index
     * Data can be read as a 16x26 2D array
     * room row * 32 vertical unit
     * room column * 2 horizontal unit
     * 3 byte per unit
     * 2 byte packet header
     */
    base_offset = ((18 - level) * 32 + (room - 5) * 2) * 3 + 2;

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    first_window.pixels[0].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    first_window.pixels[1].set(r, g, b);

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    second_window.pixels[0].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    second_window.pixels[1].set(r, g, b);

    //---------------------------------

    // Byte per row = 8 room per row * 6 byte per window row
    running_offset = 48;

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    first_window.pixels[2].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    first_window.pixels[3].set(r, g, b);

    r = (buff[(base_offset + running_offset)] & 0xf0);
    g = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    b = (buff[(base_offset + running_offset)] & 0xf0);
    second_window.pixels[2].set(r, g, b);

    r = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    g = (buff[(base_offset + running_offset)] & 0xf0);
    b = (buff[(base_offset + running_offset++)] & 0x0f) << 5;
    second_window.pixels[3].set(r, g, b);
  }
}

void network::fetch_frame() {
  if (getSn_RX_RSR(broadcast_socket) > 0 || level_number == 0 ||
      room_number == 0)
    fetch_frame_broadcast_proto();

  if (getSn_RX_RSR(unicast_socket) > 0) fetch_frame_unicast_proto();
}

void network::do_remote_command() {
  toogle_gpio(LED_COMM);

  std::uint8_t buff[32]{};
  std::uint8_t resp_addr[4];
  std::uint16_t resp_port;

  auto size{
      recvfrom(command_socket, buff, sizeof(buff), resp_addr, &resp_port)};

  // Handle too small and incorrect packages
  if (buff[0] != 'S' || buff[1] != 'E' || buff[2] != 'M' || size < 4) return;

  /*
   * When the 5th bit is set to 1 it means we're sending a broadcast command to
   * only one device
   * Can be used when the device doesn't have an IP address
   */
  if (buff[4] == 1) {
    if (netInfo.mac[0] != buff[5] || netInfo.mac[1] != buff[6] ||
        netInfo.mac[2] != buff[7] || netInfo.mac[3] != buff[8] ||
        netInfo.mac[4] != buff[9] || netInfo.mac[5] != buff[10])
      return;  // return when the MAC address doesn't match

    // If the device's IP is 0.0.0.0 use broadcast target address
    if (!netInfo.ip[0] && !netInfo.ip[1] && !netInfo.ip[2] && !netInfo.ip[3])
      resp_addr[0] = resp_addr[1] = resp_addr[2] = resp_addr[3] = 255;
  }

  switch (buff[3]) {
    case use_external_anim:
      window::internal_animation_on = false;
      break;
    case use_internal_anim:
      window::internal_animation_on = true;
      break;
    case blank:
      window::get_left_window().blank();
      window::get_right_window().blank();
      break;
    case turn_12v_off_left:
      window::get_window(window::LEFT).set_state(window::vcc_12v_off);
      break;
    case turn_12v_off_right:
      window::get_window(window::RIGHT).set_state(window::vcc_12v_off);
      break;
    case reset_left_panel:
      window::get_window(window::LEFT).set_state(window::discharge_caps);
      break;
    case reset_right_panel:
      window::get_window(window::RIGHT).set_state(window::discharge_caps);
      break;
    case reboot:
      NVIC_SystemReset();
      break;
    case get_status:
      sendto(command_socket, (std::uint8_t *)status_string,
             create_status_string(), resp_addr, resp_port);
      break;
    case get_mac:
      char mac[18];
      std::snprintf(mac, sizeof(mac), "%x:%x:%x:%x:%x:%x", netInfo.mac[0],
                    netInfo.mac[1], netInfo.mac[2], netInfo.mac[3],
                    netInfo.mac[4], netInfo.mac[5]);
      sendto(command_socket, (std::uint8_t *)mac, 17, resp_addr, resp_port);
      break;
    case delete_network_buffers:
      flush_buffers();
      break;
    case ping:
      sendto(command_socket, (std::uint8_t *)"pong", 4, resp_addr, resp_port);
      break;
    case start_firmware_update: {
      void *f = (std::uint32_t *)&_firmware_update_handler;
      goto *f;
      break;
    }
    case get_firmware_checksum: {
      auto crc = HAL_CRC_Calculate(&hcrc, (std::uint32_t *)FLASH_BASE,
                                   main_program_size);
      /* __REV for endianness fix
       * negate(crc XOR 0xFFFFFFFF) for standard CRC32
       */
      crc = __REV(~crc);

      sendto(command_socket, (std::uint8_t *)&crc, sizeof(crc), resp_addr,
             resp_port);
      break;
    }
    case swap_windows:
      window::swap_windows();
      break;
    case set_whitebalance:
      for (int i = 1; i < 22; i++) {
        window::get_window(window::LEFT).whitebalance_data[i] = buff[10 + i];
        window::get_window(window::RIGHT).whitebalance_data[i] = buff[10 + i];
      }
      window::get_window(window::LEFT).whitebalance_flag = true;
      window::get_window(window::RIGHT).whitebalance_flag = true;
      break;
    default:
      break;
  }
}
