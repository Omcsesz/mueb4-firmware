/*
 * network.c
 *
 *  Created on: Apr 7, 2018
 *      Author: kisada
 */

#include "network.hpp"

#include <dhcp.h>
#include <socket.h>
#include <w5500.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "dhcp_buffer.h"
#include "firm_update.hpp"
#include "gpios.h"
#include "mac_eeprom.h"
#include "main.h"
#include "status.hpp"
#include "stm32_flash.hpp"
#include "version.hpp"
#include "window.hpp"

namespace {
constexpr std::uint8_t command_socket{0};
constexpr std::uint8_t unicast_socket{1};
constexpr std::uint8_t multicast_socket{2};
constexpr std::uint8_t fw_update_socket{3};
constexpr std::uint8_t dhcp_socket{7};
wiz_NetInfo netInfo;

void cs_sel() {
  reset_gpio(SPI1_NSS);  // ChipSelect to low
}

void cs_desel() {
  set_gpio(SPI1_NSS);  // ChipSelect to high
}

std::uint8_t spi_rb(void) {
  while (LL_SPI_IsActiveFlag_BSY(SPI1))
    ;

  while (LL_SPI_IsActiveFlag_RXNE(SPI1))
    LL_SPI_ReceiveData8(SPI1);  // flush any FIFO content

  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    ;

  LL_SPI_TransmitData8(SPI1, 0xFF);  // send dummy byte

  while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
    ;

  return (LL_SPI_ReceiveData8(SPI1));
}

void spi_wb(std::uint8_t b) {
  while (LL_SPI_IsActiveFlag_BSY(SPI1))
    ;

  LL_SPI_TransmitData8(SPI1, b);
  LL_SPI_ReceiveData8(SPI1);
}

////////////   Status string
char status_string[512];

std::size_t create_status_string() {
  int ret;

  ret =
      std::snprintf((char *)status_string, sizeof(status_string),
                    "MUEB FW version: %s\n"
                    "MUEB MAC: %x:%x:%x:%x:%x:%x\n"
                    "anim_source: %#x\n"
                    "telemetry_comm_buff: %#x\n"
                    "frame_ether_buff: %#x\n"
                    "SEM forever\n",
                    mueb_version, netInfo.mac[0], netInfo.mac[1],
                    netInfo.mac[2], netInfo.mac[3], netInfo.mac[4],
                    netInfo.mac[5], status::if_internal_animation_is_on,
                    getSn_RX_RSR(command_socket), getSn_RX_RSR(unicast_socket));

  return (ret >= 0) ? ret : 1;
}

////////////   FW Update

bool is_update_enabled = false;

inline void enable_update_scoket() {
  LL_RCC_HSI_Enable();

  while (!LL_RCC_HSI_IsReady())
    ;

  socket(fw_update_socket, Sn_MR_TCP, 1997, 0x00);
  is_update_enabled = true;

  listen(fw_update_socket);

  std::uint8_t blocking = SOCK_IO_NONBLOCK;
  ctlsocket(fw_update_socket, CS_SET_IOMODE, &blocking);
}

void step_update() {
  static std::size_t next_page_to_fetch = 0;

  if (!is_update_enabled) return;

  std::uint8_t status;
  getsockopt(fw_update_socket, SO_STATUS, &status);
  if (status != SOCK_ESTABLISHED) return;

  std::array<std::uint8_t, 1024> buff;

  auto buffer_state = recv(fw_update_socket, buff.data(), 1024);

  if (buffer_state == SOCK_BUSY) return;

  stm32_flash::reprogramPage(buff, 32 + next_page_to_fetch);

  next_page_to_fetch++;

  if (next_page_to_fetch == 32) {  // check overflow
    disconnect(fw_update_socket);
    is_update_enabled = false;
    next_page_to_fetch = 0;
    do {
      std::uint8_t status;
      getsockopt(fw_update_socket, SO_STATUS, &status);
    } while (status == SOCK_ESTABLISHED);

    close(fw_update_socket);
  }
}

std::size_t calc_new_fw_chksum() {
  int ret;

  ret = std::snprintf(
      status_string, sizeof(status_string),
      "MUEB FW version: %s\n"
      "MUEB MAC: %x:%x:%x:%x:%x:%x\n"
      "Chksum: %u\n"
      "SEM forever\n",
      mueb_version, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2],
      netInfo.mac[3], netInfo.mac[4], netInfo.mac[5],
      static_cast<unsigned>(firmware_update::checksum_of_new_fw()));

  return (ret >= 0) ? ret : 1;
}

void fetch_frame_unicast_proto() {
  auto size = getSn_RX_RSR(unicast_socket);

  if (size == 0) return;

  status::turn_internal_anim_off();

  toogle_gpio(LED_COMM);

  std::uint8_t buff[5]{};
  size = sizeof(buff);
  std::uint8_t svr_addr[4];
  std::uint16_t svr_port;

  if (recvfrom(unicast_socket, buff, size, svr_addr, &svr_port) < size ||
      buff[0] >= 2 || buff[1] >= 4)
    return;

  bool window = buff[0];
  std::uint8_t pixel_num = buff[1];
  std::uint8_t red = buff[2];
  std::uint8_t green = buff[3];
  std::uint8_t blue = buff[4];

  status::getWindow(static_cast<status::window_from_outside>(window))
      .pixels[pixel_num]
      .set(red, green, blue);
}

void fetch_frame_multicast_proto() {
  auto size = getSn_RX_RSR(multicast_socket);
  const std::uint8_t szint = status::emelet_szam;
  const std::uint8_t szoba = status::szoba_szam;

  if (size == 0 || szint == 0 || szoba == 0) return;

  status::turn_internal_anim_off();

  toogle_gpio(LED_COMM);

  std::uint8_t buff[1500]{};
  std::uint8_t svr_addr[4];
  std::uint16_t svr_port;

  size = recvfrom(multicast_socket, buff, sizeof(buff), svr_addr, &svr_port);

  if ((buff[0] != 0x01 && buff[0] != 0x02) || (buff[0] == 0x01 && size < 314) ||
      (buff[0] == 0x02 && size < 1250))
    return;

  auto &first_window = status::getWindow(status::LEFT);
  auto &second_window = status::getWindow(status::RIGHT);

  std::uint8_t r, g, b;

  std::uint32_t base_offset = 0;
  std::size_t running_offset = 0;

  if (buff[0] == 0x01) {
    /* Read compressed pixels window wise from frame buffer
     * Data can be read as a 32x26 divided 2D array
     * One frame is divided into 4 packets
     * room row * 16(8 room per row * 2 pixel horizontally)
     * room column * 2 pixel vertically
     * 52 window per packet
     */
    std::uint8_t pn_expected = (((18 - szint) * 16 + (szoba - 5) * 2) / 52);
    std::uint8_t pn = buff[1];

    if (pn != pn_expected) return;

    /* Calculate frame buffer offset from room index
     * room row * room per row
     * room column
     * 26 room per packet(2 window per room, 52 / 2)
     * 12 byte per window
     * 2 byte packet header
     */
    base_offset = (((18 - szint) * 8 + (szoba - 5)) % 26) * 12 + 2;

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
    base_offset = ((18 - szint) * 32 + (szoba - 5) * 2) * 3 + 2;

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

void fetch_frame() {
  fetch_frame_multicast_proto();
  fetch_frame_unicast_proto();
}

}  // namespace

/*********************************
 *  network Class function defs
 ********************************/

network::network() {
  // Hard-reset W5500
  LL_GPIO_ResetOutputPin(W5500_RESET_GPIO_Port, W5500_RESET_Pin);
  HAL_Delay(1);  // min reset cycle 500 us
  LL_GPIO_SetOutputPin(W5500_RESET_GPIO_Port, W5500_RESET_Pin);
  HAL_Delay(2);  // PLL lock 1 ms max (refer datasheet)

  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);

  getMAC(netInfo.mac);
  setSHAR(netInfo.mac);

  // DHCP 1s timer located in stm32f0xx_it.c
  DHCP_init(dhcp_socket, gDATABUF);

  socket(command_socket, Sn_MR_UDP, 2000, 0x00);
  socket(unicast_socket, Sn_MR_UDP, 3000, 0x00);
  socket(multicast_socket, Sn_MR_UDP, 10000, 0x00);
}

void network::step_network() {
  if (wizphy_getphylink() == PHY_LINK_ON) {
    set_gpio(LED_JOKER);

    // Can be used without IP address
    do_remote_command();

    // do DHCP task
    switch (DHCP_run()) {
      case DHCP_IP_ASSIGN:
      case DHCP_IP_CHANGED:
      case DHCP_IP_LEASED:
        set_gpio(LED_DHCP);

        wizchip_getnetinfo(&netInfo);
        status::emelet_szam = netInfo.ip[2];
        status::szoba_szam = netInfo.ip[3];

        // Must have IP address to work
        step_update();
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
  }
}

void network::do_remote_command() {
  auto size = getSn_RX_RSR(command_socket);

  if (size == 0) return;

  toogle_gpio(LED_COMM);

  std::uint8_t buff[32]{};
  std::uint8_t resp_addr[4];
  std::uint16_t resp_port;

  size = recvfrom(command_socket, buff, sizeof(buff), resp_addr, &resp_port);

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

    // If the IP is 0.0.0.0 use broadcast target address
    if (!netInfo.ip[0] && !netInfo.ip[1] && !netInfo.ip[2] && !netInfo.ip[3])
      resp_addr[0] = resp_addr[1] = resp_addr[2] = resp_addr[3] = 255;
  }

  switch (buff[3]) {
    case use_external_anim:
      status::turn_internal_anim_off();
      break;
    case use_internal_anim:
      status::turn_internal_anim_on();
      break;
    case blank:
      status::getWindow(status::LEFT).blank();
      status::getWindow(status::RIGHT).blank();
      break;
    case turn_12v_off_left:
      status::getWindow(status::LEFT).set_state(windows::window::vcc_12v_off);
      break;
    case turn_12v_off_right:
      status::getWindow(status::RIGHT).set_state(windows::window::vcc_12v_off);
      break;
    case reset_left_panel:
      status::getWindow(status::LEFT)
          .set_state(windows::window::discharge_caps);
      break;
    case reset_right_panel:
      status::getWindow(status::RIGHT)
          .set_state(windows::window::discharge_caps);
      break;
    case reboot:
      NVIC_SystemReset();
      break;
    case get_status:
      sendto(command_socket, (std::uint8_t *)status_string,
             create_status_string(), resp_addr, resp_port);
      break;
    case get_mac:
      char mac[17];
      std::snprintf(mac, sizeof(mac), "%x:%x:%x:%x:%x:%x", netInfo.mac[0],
                    netInfo.mac[1], netInfo.mac[2], netInfo.mac[3],
                    netInfo.mac[4], netInfo.mac[5]);
      sendto(command_socket, (std::uint8_t *)mac, sizeof(mac), resp_addr,
             resp_port);
      break;
    case delete_anim_network_buffer:
      /// To be implemented TODO
      break;
    case ping:
      sendto(command_socket, (std::uint8_t *)"pong", 4, resp_addr, resp_port);
      break;
    case enable_update:
      enable_update_scoket();
      break;
    case get_new_fw_chksum:
      sendto(command_socket, (std::uint8_t *)status_string,
             calc_new_fw_chksum(), resp_addr, resp_port);
      break;
    case refurbish:
      firmware_update::refurbish();
      break;
    case swap_windows:
      status::swap_windows();
      break;
    case set_whitebalance:
      for (int i = 0; i < 21; i++) {
        status::getWindow(status::LEFT).whitebalance_data[i] = buff[11 + i];
        status::getWindow(status::RIGHT).whitebalance_data[i] = buff[11 + i];
      }
      status::getWindow(status::LEFT).set_whitebalance_flag(true);
      status::getWindow(status::RIGHT).set_whitebalance_flag(true);
      break;
    default:
      break;
  }
}
