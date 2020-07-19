#include <socket.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f0xx.h>
#include <w5500.h>

#include "gpios.h"
#include "main.h"

void cs_sel() {
  reset_gpio(SPI1_NSS);  // ChipSelect to low
}

void cs_desel() {
  set_gpio(SPI1_NSS);  // ChipSelect to high
}

uint8_t spi_rb(void) {
  while (LL_SPI_IsActiveFlag_RXNE(SPI1))
    LL_SPI_ReceiveData8(SPI1);  // flush any FIFO content

  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    ;

  LL_SPI_TransmitData8(SPI1, 0xFF);  // send dummy byte

  while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
    ;

  return (LL_SPI_ReceiveData8(SPI1));
}

void spi_wb(uint8_t b) {
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    ;

  LL_SPI_TransmitData8(SPI1, b);
}

void firmware_update() {
  __disable_irq();

  LL_RCC_HSI_Enable();

  while (!LL_RCC_HSI_IsReady())
    ;

  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);

  socket(3, Sn_MR_TCP, 1997, 0x00);

  if (listen(3) != SOCK_OK) return;

  if (HAL_FLASH_Unlock() != HAL_OK) return;

  uint8_t status;
  do {
    getsockopt(3, SO_STATUS, &status);
  } while (status != SOCK_ESTABLISHED);

  uint32_t base_addr = FLASH_BASE;

  FLASH_EraseInitTypeDef pEraseInit = {.NbPages = 56,
                                       .PageAddress = base_addr,
                                       .TypeErase = FLASH_TYPEERASE_PAGES};
  uint32_t PageError;
  if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) return;

  int32_t recvsize = 0;
  do {
    uint8_t buf[FLASH_PAGE_SIZE] = {};

    if ((recvsize = recv(3, buf, FLASH_PAGE_SIZE)) > 0) {
      int i;
      for (i = 0; i < recvsize - 1; i += 2) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr + i,
                              buf[i + 1] << 8 | buf[i]) != HAL_OK)
          return;
      }

      if (recvsize % 2 != 0) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr + i,
                              buf[i]) != HAL_OK)
          return;
      }

      base_addr += recvsize;
    }

    getsockopt(3, SO_STATUS, &status);
  } while (status != SOCK_CLOSED);

  HAL_NVIC_SystemReset();
}
