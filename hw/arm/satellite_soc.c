#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/arm/boot.h"
#include "exec/address-spaces.h"
#include "hw/arm/satellite_soc.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "qemu/units.h"
#include "hw/ssi/pl022.h"

#define REG_MASK                0xFFFF
#define REG_EXTMEM_CTRL         0x0
#define REG_EXTMEM2_CTRL        0x30
#define REG_EXTMEM3_CTRL        0x34
#define REG_EXTMEM4_CTRL        0x38
#define REG_DMA_INTR_FLAGS      0x70
#define REG_ALT_FUNCTION_CTRL   0x74 // - 0x94
#define REG_ALIAS_CTRL          0xAC
#define REG_GLOBAL_RESET        0xBC

#define INTERNAL_BANK_CNT       2
#define INTERNAL_BANK_SIZE      8 * 8 * KiB
#define EXTERNAL_BANK_CNT       4
#define EXTERNAL_BANK_SIZE      2 * 8 * MiB

#define ALIAS_CTRL_VALUES_NUM   8
#define ALIAS_CTRL_MAX_VALUE    0xC

static void SATELLITE_reset(DeviceState *dev);

static uint64_t SATELLITE_read(void *opaque, hwaddr addr, unsigned int size)
{
    SATELLITEState *s = opaque;
    uint64_t val = 0;

    addr &= REG_MASK;
    switch (addr) {
    case REG_EXTMEM_CTRL:
        val = s->external_memory_ctrl1;
        printf("%s\n", "EBOLDA\n\n");
        break;
    case REG_EXTMEM2_CTRL:
        val = s->external_memory_ctrl2;
        break;
    case REG_EXTMEM3_CTRL:
        val = s->external_memory_ctrl3;
        break;
    case REG_EXTMEM4_CTRL:
        val = s->external_memory_ctrl4;
        break;
    case REG_DMA_INTR_FLAGS:
        val = s->dma_internal_flags;
        break;
    case REG_ALT_FUNCTION_CTRL:
        val = s->gpio_alt_func_ctrl;
        break;
    case REG_ALIAS_CTRL:
        val = s->alias_ctrl;
        break;
    case REG_GLOBAL_RESET:
        val = s->global_reset;
        break;
    default:
        break;
    }

    return val;
}

static void SATELLITE_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned int size)
{
    SATELLITEState *s = opaque;

    addr &= REG_MASK;
    switch (addr) {
    case REG_EXTMEM_CTRL:
        s->external_memory_ctrl1 = val;
        break;
    case REG_EXTMEM2_CTRL:
        s->external_memory_ctrl2 = val;
        break;
    case REG_EXTMEM3_CTRL:
        s->external_memory_ctrl3 = val;
        break;
    case REG_EXTMEM4_CTRL:
        s->external_memory_ctrl4 = val;
        break;
    case REG_DMA_INTR_FLAGS:
        s->dma_internal_flags = val;
        break;
    case REG_ALT_FUNCTION_CTRL:
        s->gpio_alt_func_ctrl = val;
        break;
    case REG_ALIAS_CTRL: //переписать этот case
        s->alias_ctrl = val;

        if (val > ALIAS_CTRL_MAX_VALUE) {
            break;
        }

        uint32_t index = val;
        if (val > 3) {
            index = (index >> 2) + 4;
        }
        memory_region_set_enabled(s->aliases.region[s->aliases.last_opened_reg], false);
        memory_region_set_enabled(s->aliases.region[index], true);
        s->aliases.last_opened_reg = index;
        break;

    case REG_GLOBAL_RESET:
        s->global_reset = val;
        if (val) {
            SATELLITE_reset(opaque);
            return;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps SATELLITE_ops = {
    .read = SATELLITE_read,
    .write = SATELLITE_write
};

static void SATELLITE_reset(DeviceState *dev)
{
    SATELLITEState *s = SATELLITE_SOC(dev);

    s->external_memory_ctrl1 = 0x1FF;
    s->external_memory_ctrl2 = 0x1FF;
    s->external_memory_ctrl3 = 0x1FF;
    s->external_memory_ctrl4 = 0x1FF;
    s->dma_internal_flags = 0;
    s->gpio_alt_func_ctrl = 0;
    s->alias_ctrl = 0;
    s->global_reset = 0;

    memory_region_set_enabled(s->aliases.region[s->aliases.last_opened_reg], false);
    memory_region_set_enabled(s->aliases.region[4], true);
    s->aliases.last_opened_reg = 4;
    //TODO: make it easier
}

static void SATELLITE_soc_initfn(Object *obj)
{
    SATELLITEState *s = SATELLITE_SOC(obj);

    object_initialize_child(obj, "armv6m", &s->cpu, TYPE_ARMV7M);

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

static void SATELLITE_soc_realize(DeviceState *dev_soc, Error **errp)
{
    SATELLITEState *s = SATELLITE_SOC(dev_soc);
    DeviceState *cpu;

    MemoryRegion *system_memory = get_system_memory();

    
    if (clock_has_source(s->refclk)) {
        error_setg(errp, "refclk clock must not be wired up by the board code");
        return;
    }

    if (!clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must be wired up by the board code");
        return;
    }

    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);

    for (uint32_t i = 0; i < EXTERNAL_BANK_CNT; i++) {
        char name[32];
        snprintf(name, sizeof(name), "ChipSelect%u", i);

        s->external_mem[i] = g_new(MemoryRegion, 1);
        memory_region_init_ram(s->external_mem[i], NULL, name, EXTERNAL_BANK_SIZE,
                               &error_fatal);
        memory_region_add_subregion(system_memory, 0x8000000 + EXTERNAL_BANK_SIZE * i,
                                    s->external_mem[i]);
    }

    for (uint32_t i = 0; i < INTERNAL_BANK_CNT; i++) {
        char name[32];
        snprintf(name, sizeof(name), "IMU%u", i);

        s->internal_mem[i] = g_new(MemoryRegion, 1);
        memory_region_init_ram(s->internal_mem[i], NULL, name, INTERNAL_BANK_SIZE,
                               &error_fatal);
        memory_region_add_subregion(system_memory, 0x20000000 + INTERNAL_BANK_SIZE * i,
                                    s->internal_mem[i]);
    }

    for (uint32_t i = 0; i < (ALIAS_CTRL_VALUES_NUM - 1); i++) {
        char name[32];
        snprintf(name, sizeof(name), "ALIASING%u", i);
        MemoryRegion* current_mem_reg;
        uint32_t mem_size;

        s->aliases.region[i] = g_new(MemoryRegion, 1);
        if (i < 2) {
            current_mem_reg = s->internal_mem[i];
            mem_size = INTERNAL_BANK_SIZE;
        } else if (i == 2) {
            current_mem_reg = s->internal_mem[0];
            mem_size = INTERNAL_BANK_SIZE * 2 + 1;
        } else {
            current_mem_reg = s->external_mem[i - 3];
            mem_size = EXTERNAL_BANK_SIZE;
        }
        memory_region_init_alias(s->aliases.region[i], NULL, name, current_mem_reg, 0x0, mem_size);
        memory_region_add_subregion_overlap(system_memory, 0x0, s->aliases.region[i], 0);
        memory_region_set_enabled(s->aliases.region[i], false);
    }

#if 0
    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "SATELLITE.flash",
                           FLASH_SIZE, &error_fatal);
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "SATELLITE.flash.alias", &s->flash, 0, FLASH_SIZE);
    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);


    memory_region_init_ram(&s->sram, NULL, "SATELLITE.sram", SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, &s->sram);
#endif

    /* Init cpu */
    cpu = DEVICE(&s->cpu);
    qdev_prop_set_uint32(cpu, "num-irq", 32);
    qdev_prop_set_string(cpu, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(cpu, "enable-bitband", false);
    qdev_connect_clock_in(cpu, "cpuclk", s->sysclk);
    qdev_connect_clock_in(cpu, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->cpu), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->cpu), errp)) {
        return;
    }

    s->iomem = g_new(MemoryRegion, 1);
    memory_region_init_io(s->iomem, NULL, &SATELLITE_ops, s,
                          "SATELLITE_iomem", 0x10000);
    memory_region_add_subregion(system_memory, 0xA0000000, s->iomem);

    create_unimplemented_device("GPIOA", 0x80010000, 0xFFF);
    create_unimplemented_device("GPIOB", 0x80020000, 0xFFF);
    create_unimplemented_device("GPIOC", 0x80030000, 0xFFF);
    create_unimplemented_device("GPIOD", 0x80040000, 0xFFF);
    create_unimplemented_device("GPIOE", 0x80050000, 0xFFF);
    create_unimplemented_device("GPIOF", 0x80060000, 0xFFF);
    create_unimplemented_device("GPIOG", 0x80070000, 0xFFF);
    create_unimplemented_device("GPIOH", 0x80080000, 0xFFF);
    create_unimplemented_device("GPIOI", 0x80090000, 0xFFF);
    //create_unimplemented_device("general_purpose_registers", 0xA0000000, 0xFFFF);       ??
    //create_unimplemented_device("DMAC", 0xA0010000, 0xFFF);       ??
    create_unimplemented_device("SPI1", 0xA0020000, 0xFFF);
    create_unimplemented_device("SPI2", 0xA0030000, 0xFFF);
    create_unimplemented_device("UART1", 0xA0040000, 0xFFF);
    create_unimplemented_device("UART2", 0xA0050000, 0xFFF);
    create_unimplemented_device("UART3", 0xA0060000, 0xFFF);
    create_unimplemented_device("UART4", 0xA0070000, 0xFFF);
    create_unimplemented_device("watchdog", 0xA0080000, 0xFFF);
    create_unimplemented_device("timer1",  0xA0090000, 0xFFF);
    create_unimplemented_device("timer2", 0xA0130000, 0xFFFF);
    create_unimplemented_device("timer3", 0xA0140000, 0xFFFF);
    create_unimplemented_device("timer4", 0xA0150000, 0xFFFF);
    create_unimplemented_device("CAN1", 0xA01B0000, 0xFFFF);
    create_unimplemented_device("CAN2", 0xA01C0000, 0xFFFF);
    //create_unimplemented_device("IRQ_multiplexer", 0xA01D0000, 0xFFFF);       ??
    create_unimplemented_device("UART5", 0xA01E0000, 0xFFFF);
    create_unimplemented_device("UART6", 0xA01F0000, 0xFFFF);
    //create_unimplemented_device("I2C", 0xA0200000, 0xFFFF);       ??
    create_unimplemented_device("test_access_to_mem1_data", 0x60000000, 0xFFFF);
    create_unimplemented_device("test_access_to_mem1_ECC", 0x60100000, 0xFFFF);
    create_unimplemented_device("test_access_to_cacheWay1_data", 0x61000000, 0x3FFF);
    create_unimplemented_device("test_access_to_cacheWay1TarCrc", 0x61100000, 0x3FFF);
    create_unimplemented_device("test_access_to_mem2_data", 0x62000000, 0xFFFF);
    create_unimplemented_device("test_access_to_mem2_ECC", 0x62100000, 0xFFFF);
}

static Property SATELLITE_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", SATELLITEState, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void SATELLITE_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = SATELLITE_soc_realize;
    dc->reset = SATELLITE_reset;
    device_class_set_props(dc, SATELLITE_soc_properties);
}

static const TypeInfo SATELLITE_soc_info = {
    .name          = TYPE_SATELLITE_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SATELLITEState),
    .instance_init = SATELLITE_soc_initfn,
    .class_init    = SATELLITE_soc_class_init,
};

static void SATELLITE_soc_types(void)
{
    type_register_static(&SATELLITE_soc_info);
}

type_init(SATELLITE_soc_types)
