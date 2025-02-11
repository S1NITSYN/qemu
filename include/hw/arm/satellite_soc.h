#ifndef HW_ARM_SATELLITE_SOC_H
#define HW_ARM_SATELLITE_SOC_H

#include "hw/char/stm32f2xx_usart.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "hw/clock.h"
#include "hw/char/serial.h"

#define TYPE_SATELLITE_SOC "satellite-soc"
OBJECT_DECLARE_SIMPLE_TYPE(SATELLITEState, SATELLITE_SOC)

#define STM_NUM_USARTS 3
#define STM_NUM_SPIS 2

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE (128 * 1024)
#define SRAM_BASE_ADDRESS 0x20000000
#define SRAM_SIZE (8 * 1024)

typedef struct Memory_aliases_t {
    MemoryRegion* region[7];
    uint8_t last_opened_reg;    
} Memory_aliases_t;

struct SATELLITEState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    char *cpu_type;

    ARMv7MState cpu;

    uint32_t external_memory_ctrl1;
    uint32_t external_memory_ctrl2;
    uint32_t external_memory_ctrl3;
    uint32_t external_memory_ctrl4;
    uint32_t dma_internal_flags;
    uint32_t gpio_alt_func_ctrl;
    uint32_t alias_ctrl;
    uint32_t global_reset;

    MemoryRegion* iomem;
    MemoryRegion* external_mem[4];
    Memory_aliases_t aliases;
    MemoryRegion* internal_mem[2];

    Clock *sysclk;
    Clock *refclk;
};

#endif
