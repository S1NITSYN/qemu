#ifndef HW_TIMER_EHRPWM_H
#define HW_TIMER_EHRPWM_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "hw/ptimer.h"
#include "hw/irq.h"
#include "qom/object.h"

typedef union {
    struct {
        uint32_t ctrmode : 2;
        uint32_t phsen : 1;
        uint32_t prdld : 1;
        uint32_t syncosel : 2;
        uint32_t swfsync : 1;
        uint32_t hspclkdiv : 3;
        uint32_t clkdiv : 3;
        uint32_t phsdir : 1;
        /*
            This API configures emulation mode. This setting determines
            the behaviour of Timebase counter during emulation (debugging).
        */
        uint32_t free_soft : 2;
        uint32_t : 16;
    };
    uint32_t reg_value;
} tbctl_t;

typedef union {
    struct {
        uint32_t ctrdir : 1;
        uint32_t synci : 1;
        uint32_t ctrmax : 1;
        uint32_t : 29;
    };
    uint32_t reg_value;
} tbsts_t;

typedef struct {
    uint32_t    shadowed_value;
    uint32_t    actual_value;
} shadowable_t;

typedef struct {
    tbctl_t         tbctl;
    tbsts_t         tbsts;
    uint32_t        tbphs;
    uint32_t        tbctr;
    shadowable_t    tbprd;

    ptimer_state *ptimer;
} EHRPWMTimer;


typedef enum {
    A,
    B,
} output_signal_names;

#define TYPE_EHRPWM "EHRPWM"
OBJECT_DECLARE_SIMPLE_TYPE(EHRPWMState, EHRPWM)

struct EHRPWMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    Clock *pclk;
    EHRPWMTimer timer;

    uint32_t cmpctl;
    shadowable_t cmp[2];
    uint32_t aqctl[2];
    uint32_t aqsfrc;
    uint32_t aqcsfrc;
    uint32_t dbctl;
    uint32_t dbred;
    uint32_t dbfed;
    uint32_t tzsel;
    uint32_t tzctl;
    uint32_t tzeint;
    uint32_t tzflg;
    uint32_t tzclr;
    uint32_t tzfrc;
    uint32_t etsel;
    uint32_t etps;
    uint32_t etflg;
    uint32_t etclr;
    uint32_t etfrc;
    uint32_t pcctl;
    uint32_t fwdth;
    uint32_t tzintclr;
    uint32_t intclr;

    qemu_irq irq;
    qemu_irq EPWMx[2];
    QEMUTimer *cmp_timer[2];
};

#endif //HW_TIMER_EHRPWM_H
