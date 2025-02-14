#ifndef HW_ARM_SATELLITE_SOC_H
#define HW_ARM_SATELLITE_SOC_H

#include "hw/char/stm32f2xx_usart.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "hw/clock.h"
#include "hw/char/serial.h"
#include "hw/ssi/pl022.h"
#include "hw/char/pl011.h"
#include "hw/watchdog/cmsdk-apb-watchdog.h"
#include "hw/timer/cmsdk-apb-timer.h"
#include "hw/arm/irqmux.h"
#include "hw/gpio/cmsdk-ahb-gpio.h"

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

    Memory_aliases_t aliases;
    MemoryRegion* external_mem[4];
    MemoryRegion* internal_mem[2];

    MemoryRegion* iomem;
    uint32_t external_memory_ctrl1;
    uint32_t external_memory_ctrl2;
    uint32_t external_memory_ctrl3;
    uint32_t external_memory_ctrl4;
    uint32_t pwr_ctrl_clk;
    uint32_t pwr_ctrl_rst;
    uint32_t dma_internal_flags;
    uint32_t gpio_alt_func_ctrl;
    uint32_t alias_ctrl;
    uint32_t global_reset;

    CMSDKAHB_GPIOState gpio[9];
    //dma
    struct PL022State spi[2];
    PL011State uart[6];
    CMSDKAPBWatchdog watchdog;
    CMSDKAPBTimer timer[4];
    //can`s
    IRQMUXState multiplexer;
    //i2c ??

    Clock *sysclk;
    Clock *refclk;
};

#endif
