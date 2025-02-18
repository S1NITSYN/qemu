#ifndef HW_GPIO_CMSDKAHB_GPIO_H
#define HW_GPIO_CMSDKAHB_GPIO_H

#include "qom/object.h"
#include "hw/sysbus.h"
#include "hw/irq.h"

#define CMSDKAHB_GPIO_PIN_COUNT	16

typedef struct CMSDKAHB_GPIOState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    uint32_t data;
    uint32_t dataout;
    uint32_t outenbits;
    uint32_t altfunc;
    uint32_t inten;
    uint32_t inttype;
    uint32_t intpol;
    uint32_t intstatus;
    uint32_t masklowbyte;
    uint32_t maskhighbyte;

    qemu_irq output[CMSDKAHB_GPIO_PIN_COUNT];
    qemu_irq irqs[CMSDKAHB_GPIO_PIN_COUNT];
    
} CMSDKAHB_GPIOState;

#define TYPE_CMSDKAHB_GPIO "CMSDKAHB_GPIO"
#define CMSDKAHB_GPIO(obj) OBJECT_CHECK(CMSDKAHB_GPIOState, (obj), TYPE_CMSDKAHB_GPIO)

#endif
