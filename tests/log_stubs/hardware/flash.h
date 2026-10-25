#pragma once
#include <stdint.h>
#include <stddef.h>
#define FLASH_SECTOR_SIZE 4096U
#define FLASH_PAGE_SIZE 256U
#define PICO_FLASH_SIZE_BYTES 131072U
extern uint8_t test_flash[PICO_FLASH_SIZE_BYTES];
#define XIP_BASE ((uintptr_t)test_flash)
void flash_range_erase(uint32_t offset, size_t count);
void flash_range_program(uint32_t offset, const uint8_t *data, size_t count);
