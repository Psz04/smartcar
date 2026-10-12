#include "register.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>


/**
 * @brief 映射物理寄存器地址到用户空间。
 * @param physical_address 物理地址。
 * @param size 映射长度。
 * @retval 非 NULL 映射后的虚拟地址。
 * @retval NULL 映射失败。
 */
void *map_register(uint32_t physical_address, size_t size)
{
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        return NULL;
    }

    void *mapped = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
                        mem_fd, physical_address & ~(PAGE_SIZE - 1));
    close(mem_fd);
    if (mapped == MAP_FAILED) {
        perror("mmap register");
        return NULL;
    }

    return (void *)((uintptr_t)mapped + (physical_address & (PAGE_SIZE - 1)));
}