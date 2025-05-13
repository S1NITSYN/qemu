#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "hw/ptimer.h"
#include "hw/qdev-clock.h"

#include "hw/timer/eqep.h"
#include "hw/irq.h"
#include "qom/object.h"

#define     QPOSCNT          0x0000
#define     QPOSINIT         0x0004
#define     QPOSMAX          0x0008
#define     QPOSCMP          0x000C
#define     QPOSILAT         0x0010
#define     QPOSSLAT         0x0014
#define     QPOSLAT          0x0018
#define     QUTMR            0x001C
#define     QUPRD            0x0020
#define     QWDTMR           0x0024
#define     QWDPRD           0x0028
#define     QDECCTL          0x002C
#define     QEPCTL           0x0030
#define     QCAPCTL          0x0034
#define     QPOSCTL          0x0038
#define     QEINT            0x003C
#define     QFLG             0x0040
#define     QCLR             0x0044
#define     QFRC             0x0048
#define     QEPSTS           0x004C
#define     QCTMR            0x0050
#define     QCPRD            0x0054
#define     QCTMRLAT         0x0058
#define     QCPRDLAT         0x005C
#define     INTCLR           0x0070

#define     TIMER_MAX_VALUE  0xFFFF     

typedef enum {
    A,
    B,
} output_signal_names;

static uint32_t get_count(ptimer_state* ptimer) {
    return TIMER_MAX_VALUE - ptimer_get_count(ptimer);
}

/*static void set_count(ptimer_state* ptimer, uint32_t value) {
    ptimer_set_count(ptimer, (TIMER_MAX_VALUE - value));
}*/

/*static void EQEP_irq_update(EQEPState *s)
{
    //TODO: implement irq support
}*/

static void EQEP_timer_tick(void *opaque)
{
    EQEPState *s = (EQEPState *)opaque;

    ptimer_set_count(s->ptimer, TIMER_MAX_VALUE); //random value

    ptimer_run(s->ptimer, 1);
}

static void QEPx_handler(void *opaque, int line, int level) {
    EQEPState *s = (EQEPState *)opaque;

    ptimer_transaction_begin(s->ptimer);

    int true_line = line;
    int true_level = level;

    if (((line == A) && (s->qdecctl.qap)) ||
        ((line == B) && (s->qdecctl.qbp))) {
        true_level = (level ^ 1) & 1;
    }
    if (s->qdecctl.swap) {
        true_line = (line ^ 1) & 1;
    }

    /*
        Зависит от s->qdecctl.qsrc режима.
        Текущая реализация использовает конкретно Direction-count mode 
        AKA     s->qdecctl.qsrc = 1
    */

    if (true_line == A) {
        if ((!s->QEPx[A] && true_level) ||
            (s->QEPx[A] && !true_level && !s->qdecctl.xcr)) {
            s->qposcnt += (s->QEPx[B] ? 1 : -1);
            if (!s->qcapctl.selevnt /* && делитель соотвествует*/) { //отсутствует логика qcapctl.upps (логика делителя QCLK)
                s->qcprd = get_count(s->ptimer);
                s->qctmr = 0;
                EQEP_timer_tick(opaque);
                s->qepsts.upevnt = 1;
            }
        }
    }

    ptimer_transaction_commit(s->ptimer);
    s->QEPx[true_line] = true_level;
}

static uint64_t EQEP_read(void *opaque, hwaddr offset,
        unsigned size)
{
    EQEPState *s = (EQEPState *)opaque;
    uint32_t value = 0;

    switch (offset) {

    case QPOSCNT:
        value = s->qposcnt;
        break;
    case QPOSINIT:
        value = s->qposinit;
        break;
    case QPOSMAX:
        value = s->qposmax;
        break;
    case QPOSCMP:
        value = s->qposcmp;
        break;
    case QPOSILAT:
        value = s->qposilat;
        break;
    case QPOSSLAT:
        value = s->qposslat;
        break;
    case QPOSLAT:
        value = s->qposlat;
        break;
    case QUTMR:
        value = s->qutmr;
        break;
    case QUPRD:
        value = s->quprd;
        break;
    case QWDTMR:
        value = s->qwdtmr;
        break;
    case QWDPRD:
        value = s->qwdprd;
        break;
    case QDECCTL:
        value = s->qdecctl.reg_value;
        break;
    case QEPCTL:
        value = s->qepctl;
        break;
    case QCAPCTL:
        value = s->qcapctl.reg_value;
        break;
    case QPOSCTL:
        value = s->qposctl;
        break;
    case QEINT:
        value = s->qeint;
        break;
    case QFLG:
        value = s->qflg;
        break;
    case QCLR:
        value = s->qclr;
        break;
    case QFRC:
        value = s->qfrc;
        break;
    case QEPSTS:
        value = s->qepsts.reg_value;
        break;
    case QCTMR:
        s->qctmr = get_count(s->ptimer);
        value = s->qctmr;
        break;
    case QCPRD:
        value = s->qcprd;
        break;
    case QCTMRLAT:
        value = s->qctmrlat;
        break;
    case QCPRDLAT:
        value = s->qcprdlat;
        break;
    case INTCLR:
        value = s->intclr;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "EQEP: bad write offset " HWADDR_FMT_plx,
                      offset);
        break;
    }
    return value;
}

static void EQEP_write(void *opaque, hwaddr offset,
        uint64_t value, unsigned size)
{
    EQEPState *s = (EQEPState *)opaque;

    switch (offset) {
    case QPOSCNT:
        s->qposcnt = value;
        break;
    case QPOSINIT:
        s->qposinit = value;
        break;
    case QPOSMAX:
        s->qposmax = value;
        break;
    case QPOSCMP:
        s->qposcmp = value;
        break;
    case QPOSILAT:
        break;
    case QPOSSLAT:
        break;
    case QPOSLAT:
        break;
    case QUTMR:
        s->qutmr = value;
        break;
    case QUPRD:
        s->quprd = value;
        break;
    case QWDTMR:
        s->qwdtmr = value;
        break;
    case QWDPRD:
        s->qwdprd = value;
        break;
    case QDECCTL:
        s->qdecctl.reg_value = value;
        break;
    case QEPCTL:
        s->qepctl = value;
        break;
    case QCAPCTL:
        s->qcapctl.reg_value = value;

        ptimer_transaction_begin(s->ptimer);
        ptimer_set_period_from_clock(s->ptimer, s->pclk, 1 << s->qcapctl.ccps); //to edit
        //TODO: UPPS bits are required if any other test is to be used
        if (s->qcapctl.cen) {
            ptimer_run(s->ptimer, 1);
        } else {
            ptimer_stop(s->ptimer);
        }
        ptimer_transaction_commit(s->ptimer);

        break;
    case QPOSCTL:
        s->qposctl = value;
        break;
    case QEINT:
        s->qeint = value;
        break;
    case QFLG:
        break;
    case QCLR:
        s->qclr = value;
        break;
    case QFRC:
        s->qfrc = value;
        break;
    case QEPSTS:
        s->qepsts.reg_value = value;
        break;
    case QCTMR:
        s->qctmr = value;
        break;
    case QCPRD:
        s->qcprd = value;
        break;
    case QCTMRLAT:
        break;
    case QCPRDLAT:
        s->qcprdlat = value;
        break;
    case INTCLR:
        s->intclr = value;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "EQEP: bad write offset " HWADDR_FMT_plx,
                      offset);
        break;
    }
}

static void EQEP_reset(DeviceState *d)
{
    EQEPState *s = EQEP(d);
    
    s->qposcnt = 0;
    s->qposinit = 0;
    s->qposmax = 0;
    s->qposcmp = 0;
    s->qposilat = 0;
    s->qposslat = 0;
    s->qposlat = 0;
    s->qutmr = 0;
    s->quprd = 0;
    s->qwdtmr = 0;
    s->qwdprd = 0;
    s->qdecctl.reg_value = 0;
    s->qepctl = 0;
    s->qcapctl.reg_value = 0;
    s->qposctl = 0;
    s->qeint = 0;
    s->qflg = 0;
    s->qclr = 0;
    s->qfrc = 0;
    s->qepsts.reg_value = 0;
    s->qctmr = 0;
    s->qcprd = 0;
    s->qctmrlat = 0;
    s->qcprdlat = 0;
    s->intclr = 0;

    ptimer_transaction_begin(s->ptimer);
    ptimer_stop(s->ptimer);
    ptimer_transaction_commit(s->ptimer);
}

static const MemoryRegionOps EQEP_ops = {
    .read = EQEP_read,
    .write = EQEP_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void EQEP_clk_update(void *opaque, ClockEvent event)
{
    EQEPState *s = EQEP(opaque);

    ptimer_transaction_begin(s->ptimer);
    ptimer_set_period_from_clock(s->ptimer, s->pclk, 1);
    ptimer_transaction_commit(s->ptimer);
}

static void EQEP_init(Object *obj)
{
    EQEPState *s = EQEP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, obj, &EQEP_ops, s,
                          "EQEP", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    s->pclk = qdev_init_clock_in(DEVICE(s), "pclk",
                                 EQEP_clk_update, s, ClockUpdate);
}

static void EQEP_realize(DeviceState *dev, Error **errp)
{
    EQEPState *s = EQEP(dev);

    if (!clock_has_source(s->pclk)) {
        error_setg(errp, "EQEP: clk clock must be connected");
        return;
    }

    qdev_init_gpio_in(DEVICE(s), QEPx_handler, 2);
    qdev_init_gpio_out(DEVICE(s), &s->sync, 1);

    s->ptimer = ptimer_init(EQEP_timer_tick,
                                    s,
                                    PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);

    ptimer_transaction_begin(s->ptimer);
    ptimer_set_period_from_clock(s->ptimer, s->pclk, 1); //to edit
    ptimer_transaction_commit(s->ptimer);
}

static void EQEP_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = EQEP_realize;
    dc->reset = EQEP_reset;
}

static const TypeInfo EQEP_info = {
    .name          = TYPE_EQEP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(EQEPState),
    .instance_init = EQEP_init,
    .class_init    = EQEP_class_init,
};

static void EQEP_register_types(void)
{
    type_register_static(&EQEP_info);
}

type_init(EQEP_register_types)
