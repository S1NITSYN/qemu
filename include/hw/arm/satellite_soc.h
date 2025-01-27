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

struct SATELLITEState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    char *cpu_type;

    ARMv7MState cpu;

    /*STM32F2XXUsartState usart[STM_NUM_USARTS];
    STM32F2XXSPIState spi[STM_NUM_SPIS];*/
    //SerialState uart[6];

    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;

    Clock *sysclk;
    Clock *refclk;
};

#endif
