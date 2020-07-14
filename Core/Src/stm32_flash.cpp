#include "stm32_flash.hpp"

using namespace stm32_flash;
using stm32_flash::pageSize;

void stm32_flash::reprogramPage(const std::array<std::uint8_t, pageSize>& Buff,
                                const std::uint32_t page_num) {
  unlock_flash asd;

  erasePage(page_num);

  WaitForLastOperation();

  volatile std::uint16_t* ptr =
      reinterpret_cast<std::uint16_t*>(flash_addr + (pageSize * page_num));

  for (std::size_t i = 0; i < pageSize; i += 2) {
    std::uint16_t tmp = Buff[i + 1] << 8 | Buff[i];

    FLASH->CR |= FLASH_CR_PG;

    *ptr = tmp;
    ptr++;

    WaitForLastOperation();

    FLASH->CR &= ~FLASH_CR_PG;
  }
}

void stm32_flash::write_halfword(
    const std::uint16_t towrite,
    volatile std::uint16_t* const addr) {  // A pointer a konstans nem az adat.
  WaitForLastOperation();

  FLASH->CR |= FLASH_CR_PG;

  *addr = towrite;

  WaitForLastOperation();

  // Disable the PG Bit
  FLASH->CR &= ~FLASH_CR_PG;
}

void stm32_flash::erasePage(const std::uint32_t page_num) {
  WaitForLastOperation();

  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = reinterpret_cast<std::uint32_t>(
      reinterpret_cast<std::uint32_t>(flash_addr) +
      (pageSize * page_num));  // TODO investigate int narrowing warn
  FLASH->CR |= FLASH_CR_STRT;

  __NOP();

  WaitForLastOperation();

  FLASH->CR &= ~FLASH_CR_PER;
}
