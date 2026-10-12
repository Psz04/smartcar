#ifndef DRIVER_REGISTER_H
#define DRIVER_REGISTER_H

#include <stddef.h>
#include <stdint.h>

#define PAGE_SIZE 0x10000u
#define REG_READ(addr)      (*(volatile uint32_t *)(addr))
#define REG_WRITE(addr, v)  (*(volatile uint32_t *)(addr) = (v))

void *map_register(uint32_t physical_address, size_t size);

#endif /* DRIVER_REGISTER_H */