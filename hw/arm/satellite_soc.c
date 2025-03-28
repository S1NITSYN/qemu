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
#include "hw/char/pl011.h"
#include "hw/watchdog/cmsdk-apb-watchdog.h"
#include "hw/timer/cmsdk-apb-timer.h"
#include "hw/arm/irqmux.h"
#include "hw/gpio/cmsdk-ahb-gpio.h"
#include "hw/net/can/can_wrapper_B5023VS016.h"

#define REG_MASK                    0xFFFF
#define REG_EXTMEM_CTRL             0x0
#define REG_EDAC_CTRL               0x4
#define REG_INTMEM_CERR_CNT         0x8 
#define REG_INTMEM_FERR_CNT         0xC
#define REG_EXTMEM_CERR_CNT         0x10
#define REG_EXTMEM_FERR_CNT         0x14
#define REG_SPACEWIRE_CLK_CTRL      0x1C
#define REG_INTMEM2_CERR_CNT        0x20
#define REG_INTMEM2_FERR_CNT        0x24
#define REG_PWR_CTRL_CLK            0x28
#define REG_PWR_CTRL_RST            0x2C
#define REG_EXTMEM2_CTRL            0x30
#define REG_EXTMEM3_CTRL            0x34
#define REG_EXTMEM4_CTRL            0x38
#define REG_CACHE_HIGH_ADDR         0x3C
#define REG_INTMEM_SCR_RNG_ADDR     0x40
#define REG_INTMEM_SCR_PRD_SCAN     0x44
#define REG_INTMEM_SCR_PRD_STOP     0x48
#define REG_INTMEM2_SCR_RNG_ADDR    0x4c
#define REG_INTMEM2_SCR_PRD_SCAN    0x50
#define REG_INTMEM2_SCR_PRD_STOP    0x54
#define REG_INTMEMS_SCR_MAIN        0x58
#define REG_CACHE_CRC_ERROR         0x5c
#define REG_EDAC_INTMEM_SCR_CERR    0x60
#define REG_EDAC_INTMEM_SCR_FERR    0x64
#define REG_EDAC_INTMEM2_SCR_CERR   0x68
#define REG_EDAC_INTMEM2_SCR_FERR   0x6C
#define REG_DMA_INTR_FLAGS          0x70

/*
    В документации внутри reg_alt_func_ctrl находится 9 регистров(4 байта на каждый),
    Но при этом адрес с 0x74 по 0x94, что равно 32 байтам(вместо 36(9 * 4))
*/
//#define REG_ALT_FUNCTION_CTRL   0x74...0x94
#define REG_ALT_FUNCTION_CTRL_START   0x74 // - 0x94
#define REG_ALT_FUNCTION_CTRL_END     0x98 // в документации - 0x94
#define REG_CACHE_HIGH_ADDR_CS2         0xA0
#define REG_CACHE_HIGH_ADDR_CS3         0xA4
#define REG_CACHE_HIGH_ADDR_CS4         0xA8
#define REG_ALIAS_CTRL              0xAC
#define REG_SCRUBBER_FERR_ADDR          0xB0
#define REG_COMMON_FERR_ADDR            0xB4
#define REG_EDAC_REACTION_CTRL          0xB8
#define REG_GLOBAL_RESET            0xBC
#define REG_CACHE_MAIN                  0xC4

#define INTERNAL_BANK_CNT       2
#define INTERNAL_BANK_SIZE      (8 * 8 * KiB)
#define EXTERNAL_BANK_CNT       4
#define EXTERNAL_BANK_SIZE      (2 * 8 * MiB)

#define ALIAS_CTRL_VALUES_NUM   8
#define ALIAS_CTRL_MAX_VALUE    0xC

#define REG_ALT_FUNC_INDEX(addr)    ((addr) - REG_ALT_FUNCTION_CTRL_START) / sizeof(uint32_t)         

static void SATELLITE_reset(DeviceState *dev);

static uint64_t SATELLITE_read(void *opaque, hwaddr addr, unsigned int size)
{
    SATELLITEState *s = opaque;
    uint64_t val = 0;

    addr &= REG_MASK;
    switch (addr) {
    case REG_EXTMEM_CTRL:
        val = s->external_memory_ctrl1;
        break;
    case REG_EDAC_CTRL:
    case REG_INTMEM_CERR_CNT:
    case REG_INTMEM_FERR_CNT:
    case REG_EXTMEM_CERR_CNT:
    case REG_EXTMEM_FERR_CNT:
    case REG_SPACEWIRE_CLK_CTRL:
    case REG_INTMEM2_CERR_CNT:
    case REG_INTMEM2_FERR_CNT:
        /*UNREALIZED*/
        break;
    case REG_PWR_CTRL_CLK:
        val = s->pwr_ctrl_clk;
        break;
    case REG_PWR_CTRL_RST:
        val = s->pwr_ctrl_rst;
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
    case REG_CACHE_HIGH_ADDR:
    case REG_INTMEM_SCR_RNG_ADDR:
    case REG_INTMEM_SCR_PRD_SCAN:
    case REG_INTMEM_SCR_PRD_STOP:
    case REG_INTMEM2_SCR_RNG_ADDR:
    case REG_INTMEM2_SCR_PRD_SCAN:
    case REG_INTMEM2_SCR_PRD_STOP:
        /*UNREALIZED*/
        break;
    case REG_INTMEMS_SCR_MAIN:
        val = s->intmems_scr_main;
        break;
    case REG_CACHE_CRC_ERROR:
    case REG_EDAC_INTMEM_SCR_CERR:
    case REG_EDAC_INTMEM_SCR_FERR:
    case REG_EDAC_INTMEM2_SCR_CERR:
    case REG_EDAC_INTMEM2_SCR_FERR:
        /*UNREALIZED*/
        break;
    case REG_DMA_INTR_FLAGS:
        val = s->dma_internal_flags;
        break;
    case REG_ALT_FUNCTION_CTRL_START...REG_ALT_FUNCTION_CTRL_END:
        val = s->gpio_alt_func_ctrl[REG_ALT_FUNC_INDEX(addr)];
        break;
    case REG_CACHE_HIGH_ADDR_CS2:
    case REG_CACHE_HIGH_ADDR_CS3:
    case REG_CACHE_HIGH_ADDR_CS4:
        /*UNREALIZED*/
        break;
    case REG_ALIAS_CTRL:
        val = s->alias_ctrl;
        break;
    case REG_SCRUBBER_FERR_ADDR:
    case REG_COMMON_FERR_ADDR:
    case REG_EDAC_REACTION_CTRL:
        /*UNREALIZED*/
        break;
    case REG_GLOBAL_RESET:
        val = s->global_reset;
        break;
    case REG_CACHE_MAIN:
        /*UNREALIZED*/
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
    case REG_EDAC_CTRL:
    case REG_INTMEM_CERR_CNT:
    case REG_INTMEM_FERR_CNT:
    case REG_EXTMEM_CERR_CNT:
    case REG_EXTMEM_FERR_CNT:
    case REG_SPACEWIRE_CLK_CTRL:
    case REG_INTMEM2_CERR_CNT:
    case REG_INTMEM2_FERR_CNT:
        /*UNREALIZED*/
        break;
    case REG_PWR_CTRL_CLK:
        s->pwr_ctrl_clk = val;
        break;
    case REG_PWR_CTRL_RST:
        s->pwr_ctrl_rst = val;
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
    case REG_CACHE_HIGH_ADDR:
    case REG_INTMEM_SCR_RNG_ADDR:
    case REG_INTMEM_SCR_PRD_SCAN:
    case REG_INTMEM_SCR_PRD_STOP:
    case REG_INTMEM2_SCR_RNG_ADDR:
    case REG_INTMEM2_SCR_PRD_SCAN:
    case REG_INTMEM2_SCR_PRD_STOP:
        /*UNREALIZED*/
        break;
    case REG_INTMEMS_SCR_MAIN: 
        /*  TODO: Переделать "муляж" скраббера. Сейчас скраббер сразу "активируется"
            после записи в него стартовых значений*/
        s->intmems_scr_main = (val & 0x3) | ((val & 0x3) << 2); 
        break;
    case REG_CACHE_CRC_ERROR:
    case REG_EDAC_INTMEM_SCR_CERR:
    case REG_EDAC_INTMEM_SCR_FERR:
    case REG_EDAC_INTMEM2_SCR_CERR:
    case REG_EDAC_INTMEM2_SCR_FERR:
        /*UNREALIZED*/
        break;
    case REG_DMA_INTR_FLAGS:
        s->dma_internal_flags = val;
        break;
    case REG_ALT_FUNCTION_CTRL_START...REG_ALT_FUNCTION_CTRL_END:
        s->gpio_alt_func_ctrl[REG_ALT_FUNC_INDEX(addr)] = val;
        break;
    case REG_CACHE_HIGH_ADDR_CS2:
    case REG_CACHE_HIGH_ADDR_CS3:
    case REG_CACHE_HIGH_ADDR_CS4:
        /*UNREALIZED*/
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
    case REG_SCRUBBER_FERR_ADDR:
    case REG_COMMON_FERR_ADDR:
    case REG_EDAC_REACTION_CTRL:
        /*UNREALIZED*/
        break;
    case REG_GLOBAL_RESET:
        s->global_reset = val;
        if (val) {
            SATELLITE_reset(opaque);
            return;
        }
        break;
    case REG_CACHE_MAIN:
        /*UNREALIZED*/
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
    
    for (int i = 0; i < 9; i++) {
        s->gpio_alt_func_ctrl[i] = 0;
    }
    s->dma_internal_flags = 0;
    s->alias_ctrl = 0;
    s->global_reset = 0;

    memory_region_set_enabled(s->aliases.region[s->aliases.last_opened_reg], false);
    memory_region_set_enabled(s->aliases.region[4], true);
    s->aliases.last_opened_reg = 4;
}

static void SATELLITE_soc_initfn(Object *obj)
{
    char name[32];
    uint32_t i;

    SATELLITEState *s = SATELLITE_SOC(obj);

    object_initialize_child(obj, "armv6m", &s->cpu, TYPE_ARMV7M);

    object_initialize_child(obj, "irqmux", &s->multiplexer, TYPE_IRQMUX);

    object_initialize_child(obj, "spi0", &s->spi[0], TYPE_PL022);
    object_initialize_child(obj, "spi1", &s->spi[1], TYPE_PL022);

    for (i = 0; i < 9; i++) {
        snprintf(name, sizeof(name), "gpio%u", i);
        object_initialize_child(obj, name, &s->gpio[i],
                                TYPE_CMSDKAHB_GPIO);
    }

    for (i = 0; i < 6; i++) {
        snprintf(name, sizeof(name), "uart%u", i);
        if (serial_hd(i)) {
            object_initialize_child(obj, name, &s->uart[i], TYPE_PL011);
        } else {
            break;
        }
    }

    for (i = 0; i < 4; i++) {
        snprintf(name, sizeof(name), "timer%u", i);
        object_initialize_child(obj, name, &s->timer[i],
                                TYPE_CMSDK_APB_TIMER);
    }

    Object *can_bus = object_resolve_path("canbus0", NULL);
    if (can_bus) {
        object_initialize_child(obj, "CAN0", &s->CAN[0], TYPE_CAN_DEV);
        object_initialize_child(obj, "CAN1", &s->CAN[1], TYPE_CAN_DEV);
        object_property_set_link(OBJECT(&s->CAN[0]), "canbus", can_bus, &error_fatal);
        object_property_set_link(OBJECT(&s->CAN[1]), "canbus", can_bus, &error_fatal);
    }

    object_initialize_child(obj, "watchdog", &s->watchdog, TYPE_CMSDK_APB_WATCHDOG);

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

static void SATELLITE_soc_realize(DeviceState *dev_soc, Error **errp)
{
    SATELLITEState *s = SATELLITE_SOC(dev_soc);
    DeviceState *cpu;
    SysBusDevice *busdev;

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

    busdev = SYS_BUS_DEVICE(&s->multiplexer);
    if (!sysbus_realize(busdev, &error_fatal)) {
        return;
    }
    memory_region_add_subregion(system_memory, 0xA01D0000,
                                sysbus_mmio_get_region(busdev, 0));
    for (uint32_t i = 0; i < LINE_MAX_NUM; i++) {
        qdev_connect_gpio_out(DEVICE(&s->multiplexer), i, qdev_get_gpio_in(cpu, i));
    }


    uint32_t ALTFUNCVALS[9] = {0xFFFF, 0xFFFF, 0xFFFF, 0x007F,
                               0x0008, 0x0000, 0x0000, 0x00E0, 0x0000};
    for (uint32_t i = 0; i < 9; i++) {
        busdev = SYS_BUS_DEVICE(&s->gpio[i]);
        qdev_prop_set_uint32(DEVICE(&s->gpio[i]), "AltFuncVal", ALTFUNCVALS[i]);
        if (!sysbus_realize(busdev, &error_fatal)) {
            return;
        }
        memory_region_add_subregion(system_memory, 0x80000000 + i * 0x10000,
                                    sysbus_mmio_get_region(busdev, 0));
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->multiplexer), 8 + i));
    }

    busdev = SYS_BUS_DEVICE(&s->spi[0]);
    if (!sysbus_realize(busdev, &error_fatal)) {
        return;
    }
    memory_region_add_subregion(system_memory, 0xA0020000,
                                sysbus_mmio_get_region(busdev, 0));
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->multiplexer), 32));

#if 0
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        DeviceState *flash_dev;
        struct BlockBackend *blk = blk_by_legacy_dinfo(dinfo);

        switch (blk_getlength(blk)) {
        default:
        case 4 * MiB:
            flash_dev = qdev_new("m25p32");
            break;

        case 16 * MiB:
            flash_dev = qdev_new("n25q128a13");
            break;
        }

        qdev_prop_set_drive_err(flash_dev, "drive", blk, &error_fatal);

        // Our flash has 1 dummy cycle (or at least with this value it works)
        // So we take default value and set dummy cycles to 1
        object_property_set_int(OBJECT(flash_dev), "nonvolatile-cfg", 0x1fff,
                                &error_fatal);
        qdev_realize(flash_dev, BUS(s->spi[0].ssi), &error_fatal);

        // Connect spi_flash chip select (cs pin) to 2nd pin of gpio1
        qdev_connect_gpio_out(s->lsif1_gpio[1], 2,
                              qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
    }
#endif

    busdev = SYS_BUS_DEVICE(&s->spi[1]);
    if (!sysbus_realize(busdev, &error_fatal)) {
        return;
    }
    memory_region_add_subregion(system_memory, 0xA0030000,
                                sysbus_mmio_get_region(busdev, 0));
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->multiplexer), 33));

    for (uint32_t i = 0; i < 6; i++) {
        uint32_t device_address;

        if (serial_hd(i) && (i < 4)) {
            device_address = 0xA0040000 + i * 0x10000;
        } else if (serial_hd(i)) {
            device_address = 0xA01E0000 + (i - 4) * 0x10000;
        } else {
            break;
        }
        qdev_prop_set_chr(DEVICE(&s->uart[i]), "chardev", serial_hd(i));
        busdev = SYS_BUS_DEVICE(&s->uart[i]);
        if (!sysbus_realize(busdev, &error_fatal)) {
            return;
        }
        memory_region_add_subregion(system_memory, device_address,
                                    sysbus_mmio_get_region(busdev, 0));
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->multiplexer), 24 + i));
    }

    for (uint32_t i = 0; i < 4; i++) {
        uint32_t device_address = 0xA0090000;
        if (i > 0) {
            device_address = 0xA0120000 + 0x10000 * i;
        }

        busdev = SYS_BUS_DEVICE(&s->timer[i]);
        qdev_connect_clock_in(DEVICE(&s->timer[i]), "pclk", s->sysclk);
        if (!sysbus_realize_and_unref(busdev, &error_fatal)) {
            return;
        }
        memory_region_add_subregion(system_memory, device_address,
                                    sysbus_mmio_get_region(busdev, 0));
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->multiplexer), 1 + i));
    }

    if (object_resolve_path("canbus0", NULL)) {
        for (uint32_t i = 0; i < 2; i++) {
            busdev = SYS_BUS_DEVICE(&s->CAN[i]);
            if (!sysbus_realize(busdev, &error_fatal)) {
                return;
            }
            memory_region_add_subregion(system_memory, 0xA01B0000 + i * 0x10000,
                                        sysbus_mmio_get_region(busdev, 0));
            sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->multiplexer), 52 + i));
        }
    } else {
        create_unimplemented_device("CAN1", 0xA01B0000, 0x10000);
        create_unimplemented_device("CAN2", 0xA01C0000, 0x10000);
    }
    
    qdev_connect_clock_in(DEVICE(&s->watchdog), "WDOGCLK", s->sysclk);
    busdev = SYS_BUS_DEVICE(&s->watchdog);
    if (!sysbus_realize(busdev, &error_fatal)) {
        return;
    }
    memory_region_add_subregion(system_memory, 0xA0080000,
                                    sysbus_mmio_get_region(busdev, 0));
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->multiplexer), 0));

    //create_unimplemented_device("DMAC", 0xA0010000, 0x1000);       ??
    //create_unimplemented_device("I2C", 0xA0200000, 0x10000);       ??
    create_unimplemented_device("test_access_to_mem1_data", 0x60000000, 0x10000);
    create_unimplemented_device("test_access_to_mem1_ECC", 0x60100000, 0x10000);
    create_unimplemented_device("test_access_to_cacheWay1_data", 0x61000000, 0x4000);
    create_unimplemented_device("test_access_to_cacheWay1TarCrc", 0x61100000, 0x4000);
    create_unimplemented_device("test_access_to_mem2_data", 0x62000000, 0x10000);
    create_unimplemented_device("test_access_to_mem2_ECC", 0x62100000, 0x10000);
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
