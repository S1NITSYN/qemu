#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "qemu/units.h"
#include "hw/arm/irqmux.h"

#define IRQ_SRC_WATCHDOG 0
#define IRQ_SRC_TIMER1 1
#define IRQ_SRC_TIMER2 2
#define IRQ_SRC_TIMER3 3
#define IRQ_SRC_TIMER4 4
#define IRQ_SRC_GPIOA 8
#define IRQ_SRC_GPIOB 9
#define IRQ_SRC_GPIOC 10
#define IRQ_SRC_GPIOD 11
#define IRQ_SRC_GPIOE 12
#define IRQ_SRC_GPIOF 13
#define IRQ_SRC_GPIOG 14
#define IRQ_SRC_GPIOH 15
#define IRQ_SRC_GPIOI 16
#define IRQ_SRC_UART1 24
#define IRQ_SRC_UART2 25
#define IRQ_SRC_UART3 26
#define IRQ_SRC_UART4 27
#define IRQ_SRC_UART5 28
#define IRQ_SRC_UART6 29
#define IRQ_SRC_SPI1 32
#define IRQ_SRC_SPI2 33
#define IRQ_SRC_I2C 50
#define IRQ_SRC_CAN1 52
#define IRQ_SRC_CAN2 53
#define IRQ_SRC_DMA_SPI_1_TX 96
#define IRQ_SRC_DMA_SPI_2_RX 99
#define IRQ_SRC_DMA_UART_1_TX 100
#define IRQ_SRC_DMA_UART_1_RX 101
#define IRQ_SRC_DMA_UART_2_TX 102
#define IRQ_SRC_DMA_UART_2_RX 103
#define IRQ_SRC_DMA_UART_3_TX 104
#define IRQ_SRC_DMA_UART_3_RX 105
#define IRQ_SRC_DMA_UART_4_TX 106
#define IRQ_SRC_DMA_UART_4_RX 107
#define IRQ_SRC_DMA_UART_5_TX 108
#define IRQ_SRC_DMA_UART_5_RX 109
#define IRQ_SRC_DMA_UART_6_TX 110
#define IRQ_SRC_DMA_UART_6_RX 111

#define CTRL_REG_EN (1 << 31)
#define CTRL_REG_VAL 0x1F

#define REGS_MASK(addr) ((addr) >> 2)

#if 0
#define IRQ_SRC_DMA_CHAIN16 0
#define IRQ_SRC_DMA_CHAIN17 0
#define IRQ_SRC_DMA_CHAIN18 0
#define IRQ_SRC_DMA_CHAIN20 0
#define IRQ_SRC_DMA_CHAIN21 0
#define IRQ_SRC_DMA_CHAIN22 0
#define IRQ_SRC_DMA_CHAIN23 0
#endif

static uint64_t IRQMUX_read(void *opaque, hwaddr addr, unsigned int size) {
    IRQMUXState *s = opaque;
    uint64_t val = 0;

    switch (REGS_MASK(addr)) {
    case IRQ_SRC_WATCHDOG:
        val = s->int_mux_ctrl_regs[IRQ_SRC_WATCHDOG];
        break;
    case IRQ_SRC_TIMER1:
        val = s->int_mux_ctrl_regs[IRQ_SRC_TIMER1];
        break;
    case IRQ_SRC_TIMER2:
        val = s->int_mux_ctrl_regs[IRQ_SRC_TIMER2];
        break;
    case IRQ_SRC_TIMER3:
        val = s->int_mux_ctrl_regs[IRQ_SRC_TIMER3];
        break;
    case IRQ_SRC_TIMER4:
        val = s->int_mux_ctrl_regs[IRQ_SRC_TIMER4];
        break;
    case IRQ_SRC_GPIOA:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOA];
        break;
    case IRQ_SRC_GPIOB:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOB];
        break;
    case IRQ_SRC_GPIOC:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOC];
        break;
    case IRQ_SRC_GPIOD:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOD];
        break;
    case IRQ_SRC_GPIOE:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOE];
        break;
    case IRQ_SRC_GPIOF:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOF];
        break;
    case IRQ_SRC_GPIOG:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOG];
        break;
    case IRQ_SRC_GPIOH:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOH];
        break;
    case IRQ_SRC_GPIOI:
        val = s->int_mux_ctrl_regs[IRQ_SRC_GPIOI];
        break;
    case IRQ_SRC_UART1:
        val = s->int_mux_ctrl_regs[IRQ_SRC_UART1];
        break;
    case IRQ_SRC_UART2:
        val = s->int_mux_ctrl_regs[IRQ_SRC_UART2];
        break;
    case IRQ_SRC_UART3:
        val = s->int_mux_ctrl_regs[IRQ_SRC_UART3];
        break;
    case IRQ_SRC_UART4:
        val = s->int_mux_ctrl_regs[IRQ_SRC_UART4];
        break;
    case IRQ_SRC_UART5:
        val = s->int_mux_ctrl_regs[IRQ_SRC_UART5];
        break;
    case IRQ_SRC_UART6:
        val = s->int_mux_ctrl_regs[IRQ_SRC_UART6];
        break;
    case IRQ_SRC_SPI1:
        val = s->int_mux_ctrl_regs[IRQ_SRC_SPI1];
        break;
    case IRQ_SRC_SPI2:
        val = s->int_mux_ctrl_regs[IRQ_SRC_SPI2];
        break;
    case IRQ_SRC_I2C:
        val = s->int_mux_ctrl_regs[IRQ_SRC_I2C];
        break;
    case IRQ_SRC_CAN1:
        val = s->int_mux_ctrl_regs[IRQ_SRC_CAN1];
        break;
    case IRQ_SRC_CAN2:
        val = s->int_mux_ctrl_regs[IRQ_SRC_CAN2];
        break;
    case IRQ_SRC_DMA_SPI_1_TX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_SPI_1_TX];
        break;
    case IRQ_SRC_DMA_SPI_2_RX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_SPI_2_RX];
        break;
    case IRQ_SRC_DMA_UART_1_TX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_1_TX];
        break;
    case IRQ_SRC_DMA_UART_1_RX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_1_RX];
        break;
    case IRQ_SRC_DMA_UART_2_TX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_2_TX];
        break;
    case IRQ_SRC_DMA_UART_2_RX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_2_RX];
        break;
    case IRQ_SRC_DMA_UART_3_TX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_3_TX];
        break;
    case IRQ_SRC_DMA_UART_3_RX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_3_RX];
        break;
    case IRQ_SRC_DMA_UART_4_TX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_4_TX];
        break;
    case IRQ_SRC_DMA_UART_4_RX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_4_RX];
        break;
    case IRQ_SRC_DMA_UART_5_TX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_5_TX];
        break;
    case IRQ_SRC_DMA_UART_5_RX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_5_RX];
        break;
    case IRQ_SRC_DMA_UART_6_TX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_6_TX];
        break;
    case IRQ_SRC_DMA_UART_6_RX:
        val = s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_6_RX];
        break;
    default:
        break;
    }

    return val;
}

static void IRQMUX_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned int size) {
    IRQMUXState *s = IRQMUX(opaque);

    switch (REGS_MASK(addr)) {
    case IRQ_SRC_WATCHDOG:
        s->int_mux_ctrl_regs[IRQ_SRC_WATCHDOG] = val;
        break;
    case IRQ_SRC_TIMER1:
        s->int_mux_ctrl_regs[IRQ_SRC_TIMER1] = val;
        break;
    case IRQ_SRC_TIMER2:
        s->int_mux_ctrl_regs[IRQ_SRC_TIMER2] = val;
        break;
    case IRQ_SRC_TIMER3:
        s->int_mux_ctrl_regs[IRQ_SRC_TIMER3] = val;
        break;
    case IRQ_SRC_TIMER4:
        s->int_mux_ctrl_regs[IRQ_SRC_TIMER4] = val;
        break;
    case IRQ_SRC_GPIOA:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOA] = val;
        break;
    case IRQ_SRC_GPIOB:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOB] = val;
        break;
    case IRQ_SRC_GPIOC:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOC] = val;
        break;
    case IRQ_SRC_GPIOD:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOD] = val;
        break;
    case IRQ_SRC_GPIOE:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOE] = val;
        break;
    case IRQ_SRC_GPIOF:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOF] = val;
        break;
    case IRQ_SRC_GPIOG:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOG] = val;
        break;
    case IRQ_SRC_GPIOH:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOH] = val;
        break;
    case IRQ_SRC_GPIOI:
        s->int_mux_ctrl_regs[IRQ_SRC_GPIOI] = val;
        break;
    case IRQ_SRC_UART1:
        s->int_mux_ctrl_regs[IRQ_SRC_UART1] = val;
        break;
    case IRQ_SRC_UART2:
        s->int_mux_ctrl_regs[IRQ_SRC_UART2] = val;
        break;
    case IRQ_SRC_UART3:
        s->int_mux_ctrl_regs[IRQ_SRC_UART3] = val;
        break;
    case IRQ_SRC_UART4:
        s->int_mux_ctrl_regs[IRQ_SRC_UART4] = val;
        break;
    case IRQ_SRC_UART5:
        s->int_mux_ctrl_regs[IRQ_SRC_UART5] = val;
        break;
    case IRQ_SRC_UART6:
        s->int_mux_ctrl_regs[IRQ_SRC_UART6] = val;
        break;
    case IRQ_SRC_SPI1:
        s->int_mux_ctrl_regs[IRQ_SRC_SPI1] = val;
        break;
    case IRQ_SRC_SPI2:
        s->int_mux_ctrl_regs[IRQ_SRC_SPI2] = val;
        break;
    case IRQ_SRC_I2C:
        s->int_mux_ctrl_regs[IRQ_SRC_I2C] = val;
        break;
    case IRQ_SRC_CAN1:
        s->int_mux_ctrl_regs[IRQ_SRC_CAN1] = val;
        break;
    case IRQ_SRC_CAN2:
        s->int_mux_ctrl_regs[IRQ_SRC_CAN2] = val;
        break;
    case IRQ_SRC_DMA_SPI_1_TX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_SPI_1_TX] = val;
        break;
    case IRQ_SRC_DMA_SPI_2_RX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_SPI_2_RX] = val;
        break;
    case IRQ_SRC_DMA_UART_1_TX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_1_TX] = val;
        break;
    case IRQ_SRC_DMA_UART_1_RX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_1_RX] = val;
        break;
    case IRQ_SRC_DMA_UART_2_TX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_2_TX] = val;
        break;
    case IRQ_SRC_DMA_UART_2_RX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_2_RX] = val;
        break;
    case IRQ_SRC_DMA_UART_3_TX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_3_TX] = val;
        break;
    case IRQ_SRC_DMA_UART_3_RX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_3_RX] = val;
        break;
    case IRQ_SRC_DMA_UART_4_TX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_4_TX] = val;
        break;
    case IRQ_SRC_DMA_UART_4_RX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_4_RX] = val;
        break;
    case IRQ_SRC_DMA_UART_5_TX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_5_TX] = val;
        break;
    case IRQ_SRC_DMA_UART_5_RX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_5_RX] = val;
        break;
    case IRQ_SRC_DMA_UART_6_TX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_6_TX] = val;
        break;
    case IRQ_SRC_DMA_UART_6_RX:
        s->int_mux_ctrl_regs[IRQ_SRC_DMA_UART_6_RX] = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps IRQMUX_ops = {
    .read = IRQMUX_read,
    .write = IRQMUX_write,
};

static void IRQMUX_reset(DeviceState *dev) {
    IRQMUXState *s = IRQMUX(dev);

    for (uint32_t i = 0; i < IRQ_MAX_NUM; i++) {
        s->int_mux_ctrl_regs[i] = 0;
    }
}

static void IRQMUX_handler(void *opaque, int irq, int level) {
    IRQMUXState *s = IRQMUX(opaque);

    if (!(s->int_mux_ctrl_regs[irq] & CTRL_REG_EN)) {
        qemu_irq_lower(s->gpio_out[s->int_mux_ctrl_regs[irq] & CTRL_REG_VAL]);
        return;
    }

    if (level) {
        qemu_irq_raise(s->gpio_out[s->int_mux_ctrl_regs[irq] & CTRL_REG_VAL]);
    } else {
        qemu_irq_lower(s->gpio_out[s->int_mux_ctrl_regs[irq] & CTRL_REG_VAL]);
    }
}

static void IRQMUX_realize(DeviceState *dev, Error **errp) {
    IRQMUXState *s = IRQMUX(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &IRQMUX_ops, s, TYPE_IRQMUX,
                          0x10000);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(DEVICE(dev), IRQMUX_handler, IRQ_MAX_NUM);

    qdev_init_gpio_out(DEVICE(dev), s->gpio_out, 32);
}

static void IRQMUX_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = IRQMUX_realize;
    dc->reset = IRQMUX_reset;
    dc->desc = "ARM INTERRUPT MULTIPLEXER";
}

static const TypeInfo IRQMUX_info = {
    .name = TYPE_IRQMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IRQMUXState),
    .class_init = IRQMUX_class_init,
};

static void IRQMUX_register_types(void) {
    type_register_static(&IRQMUX_info);
}

type_init(IRQMUX_register_types)
