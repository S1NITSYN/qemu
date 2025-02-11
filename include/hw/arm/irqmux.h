#ifndef HW_ARM_IRQMUX_H
#define HW_ARM_IRQMUX_H

#include "qom/object.h"
#include "hw/sysbus.h"
#include "hw/irq.h"

#define IRQ_MAX_NUM     120
#define LINE_MAX_NUM    32

typedef struct IRQMUXState {
    /*< private >*/
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /*< public >*/
    
    qemu_irq gpio_out[LINE_MAX_NUM];
    uint32_t int_mux_ctrl_regs[IRQ_MAX_NUM];
} IRQMUXState;

#define TYPE_IRQMUX "IRQMUX"
#define IRQMUX(obj) OBJECT_CHECK(IRQMUXState, (obj), TYPE_IRQMUX)

#endif
