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

    const uint32_t EXTERNAL_BANK_SIZE = 2 * 8 * MiB;

    MemoryRegion *emu1 = g_new(MemoryRegion, 1);
    memory_region_init_ram(emu1, NULL, "ChipSelect1", EXTERNAL_BANK_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, 0x8000000 + EXTERNAL_BANK_SIZE * 0,
                                emu1);

    MemoryRegion *emu2 = g_new(MemoryRegion, 1);
    memory_region_init_ram(emu2, NULL, "ChipSelect2", EXTERNAL_BANK_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, 0x8000000 + EXTERNAL_BANK_SIZE * 1,
                                emu2);

    MemoryRegion *emu3 = g_new(MemoryRegion, 1);
    memory_region_init_ram(emu3, NULL, "ChipSelect3", EXTERNAL_BANK_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, 0x8000000 + EXTERNAL_BANK_SIZE * 2,
                                emu3);

    MemoryRegion *emu4 = g_new(MemoryRegion, 1);
    memory_region_init_ram(emu4, NULL, "ChipSelect4", EXTERNAL_BANK_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, 0x8000000 + EXTERNAL_BANK_SIZE * 3,
                                emu4);

    MemoryRegion *ALIASING = g_new(MemoryRegion, 1);
    memory_region_init_alias(ALIASING, NULL, "ALIASING", emu1, 0, EXTERNAL_BANK_SIZE);
    memory_region_add_subregion(system_memory, 0x0, ALIASING);
    
    const uint32_t INTERNAL_BANKS_CNT = 2;
    const uint32_t INTERNAL_BANK_SIZE = 8 * 8 * KiB;
    for (uint32_t i = 0; i < INTERNAL_BANKS_CNT; i++) {
        char name[32];
        snprintf(name, sizeof(name), "IMU%u", i);

        MemoryRegion *imu = g_new(MemoryRegion, 1);
        memory_region_init_ram(imu, NULL, name, INTERNAL_BANK_SIZE,
                               &error_fatal);
        memory_region_add_subregion(system_memory, 0x20000000 + INTERNAL_BANK_SIZE * i,
                                    imu);
    }

    /* Init cpu */
    cpu = DEVICE(&s->cpu);
    qdev_prop_set_uint32(cpu, "num-irq", 32);
    qdev_prop_set_string(cpu, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(cpu, "enable-bitband", false);
    qdev_connect_clock_in(cpu, "cpuclk", s->sysclk);
    qdev_connect_clock_in(cpu, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->cpu), "memory",
                             OBJECT(get_system_memory()), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->cpu), errp)) {
        return;
    }

    //create_unimplemented_device("aliasing??",  0x00000000, 0xFFFFFF);
    //create_unimplemented_device("chip_select1",  0x8000000, 0xFFFFFF);
    create_unimplemented_device("chip_select2",  0x9000000, 0xFFFFFF);
    create_unimplemented_device("chip_select3",  0xA000000, 0xFFFFFF);
    create_unimplemented_device("chip_select4",  0xB000000, 0xFFFFFF);
    //create_unimplemented_device("imu1", 0x20000000, 0xFFFF);
    //create_unimplemented_device("imu2", 0x20010000, 0xFFFF);
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
