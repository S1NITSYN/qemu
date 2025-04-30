#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "hw/ptimer.h"
#include "hw/qdev-clock.h"

#include "hw/timer/ehrpwm.h"
#include "hw/irq.h"
#include "qom/object.h"

#define     REG_TBCTL         0x0000
#define     REG_TBSTS         0x0004
#define     REG_TBPHS         0x0008
#define     REG_TBCTR         0x000C
#define     REG_TBPRD         0x0010
#define     REG_CMPCTL        0x0014
#define     REG_CMPA          0x0018
#define     REG_CMPB          0x001C
#define     REG_AQCTLA        0x0020
#define     REG_AQCTLB        0x0024
#define     REG_AQSFRC        0x0028
#define     REG_AQCSFRC       0x002C
#define     REG_DBCTL         0x0030
#define     REG_DBRED         0x0034
#define     REG_DBFED         0x0038
#define     REG_TZSEL         0x003C
#define     REG_TZCTL         0x0040
#define     REG_TZEINT        0x0044
#define     REG_TZFLG         0x0048
#define     REG_TZCLR         0x004C
#define     REG_TZFRC         0x0050
#define     REG_ETSEL         0x0054
#define     REG_ETPS          0x0058
#define     REG_ETFLG         0x005C
#define     REG_ETCLR         0x0060
#define     REG_ETFRC         0x0064
#define     REG_PCCTL         0x0068
#define     REG_FWDTH         0x0070
#define     REG_TZINTCLR      0x00A4
#define     REG_INTCLR        0x00A8

#define     MAX_TIMER_VALUE   0xFFFF

#define     TBCTL_MODE_COUNT_UP       0
#define     TBCTL_MODE_COUNT_DOWN     1
#define     TBCTL_MODE_COUNT_UP_DOWN  2
#define     TBCTL_MODE_COUNT_STOPPED  3

#define     TBSTS_MODE_COUNT_UP       1
#define     TBSTS_MODE_COUNT_DOWN     0

static void AQ_handler(void *opaque, output_signal_names cmp_index) {
    EHRPWMState *s = (EHRPWMState *)opaque;
    int offset = 0;

    if (cmp_index == A)
        offset += 4;
    else if (cmp_index == B)
        offset += 8;

    if (s->timer.tbsts.ctrdir == TBSTS_MODE_COUNT_DOWN)
        offset += 2;

    for (output_signal_names index = A; index < 2; index++) {
        switch(s->aqctl[index] & (0x3 << offset)) {
            case 0:
                printf("ВСЕ ТАК?)\t%x\n\n", index);
                break;
            case 1:
                printf("ВСЕ НЕ ТАК1?)\t%x\n\n", index);
                qemu_irq_lower(s->EPWMx[index]);
                break;
            case 2:
                printf("ВСЕ НЕ ТАК2?)\t%x\n\n", index);

                qemu_irq_raise(s->EPWMx[index]);
                break;
            case 3:
                printf("ВСЕ НЕ ТАК3?)\t%x\n\n", index);

                qemu_irq_invert(s->EPWMx[index]);
                break;
        }
    }
}

static void cmp_reload(void *opaque, output_signal_names index)
{
    EHRPWMState *s = (EHRPWMState *)opaque;
    int64_t tick = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t count = 0;
    uint32_t cmp_value = extract32(s->cmp[index].actual_value, 16, 16);
    //can write an assert here

    switch (s->timer.tbsts.ctrdir) {
    case TBSTS_MODE_COUNT_UP:
        count = cmp_value;
        break;
    case TBSTS_MODE_COUNT_DOWN:
        count = (uint16_t)s->timer.tbprd.actual_value - cmp_value;
        break;
    }

    uint32_t clkdiv_value = (1 << s->timer.tbctl.clkdiv);
    uint32_t hspclkdiv_value = 
               (!s->timer.tbctl.hspclkdiv) ? 1 : (2 * s->timer.tbctl.hspclkdiv);

    tick += clock_ticks_to_ns(s->pclk, count * clkdiv_value * hspclkdiv_value); //НАДО БУДЕТ ПРОВЕРИТЬ

    timer_mod(s->cmp_timer[index], tick);
}

static void cmp_tick_A(void *opaque)
{
    /*EHRPWMState *s = (EHRPWMState *)opaque;
    timer_del(s->cmp_timer[A]);*/ //Это пока на всякий
    AQ_handler(opaque, A); 
}

static void cmp_tick_B(void *opaque)
{
    /*EHRPWMState *s = (EHRPWMState *)opaque;
    timer_del(s->cmp_timer[B]);*/ //Это пока на всякий
    AQ_handler(opaque, B);
}

static uint16_t get_count(void *opaque) {
    EHRPWMState *s = (EHRPWMState *)opaque;
    uint16_t value = 0;
    uint8_t direction = s->timer.tbsts.ctrdir;

    if (!((uint16_t)s->timer.tbprd.actual_value) || (direction == TBSTS_MODE_COUNT_DOWN)) {
        value = ((uint16_t)ptimer_get_count(s->timer.ptimer));
    } else if (direction == TBSTS_MODE_COUNT_UP) {
        value =
         ((uint16_t)s->timer.tbprd.actual_value) - ((uint16_t)ptimer_get_count(s->timer.ptimer));
    }
    return value;
}

static void set_count(void *opaque, uint32_t value) {
    EHRPWMState *s = (EHRPWMState *)opaque;
    switch (s->timer.tbsts.ctrdir) {
    case TBSTS_MODE_COUNT_UP:
        ptimer_set_count(s->timer.ptimer,
                         ((uint16_t)s->timer.tbprd.actual_value) - ((uint16_t)value));
        break;
    case TBSTS_MODE_COUNT_DOWN:
        ptimer_set_count(s->timer.ptimer, value);
        break;
    }
}

static void EHRPWM_irq_update(EHRPWMState *s)
{
    //TODO: implement irq support
}

static void EHRPWM_tick(void *opaque)
{
    EHRPWMState *s = (EHRPWMState *)opaque;
    /*
        Если было достигнуто максмальное значение таймера (0xFFFF),
        то, нужно это отметить в статусном регистре TBSTS значением 0x4
    */

    //Где-то здесь можно будет добавить обраотку прерываний, если нужно

    if (s->timer.tbctl.ctrmode == TBCTL_MODE_COUNT_UP_DOWN) {
        s->timer.tbsts.ctrdir ^= 0x1;
    }

    if (s->timer.tbsts.ctrdir == TBSTS_MODE_COUNT_UP) {
        if (!s->timer.tbctl.prdld) {
            s->timer.tbprd.actual_value = s->timer.tbprd.shadowed_value;
            ptimer_set_limit(s->timer.ptimer, (uint16_t)s->timer.tbprd.actual_value, 0);
        }
        /*
            Следующие строчки тут находятся только при применении текущего
            теста. Если будут применяться другие тесты, то следующее поведение
            надо будет переписывать
        */
        s->cmp[A].actual_value = s->cmp[A].shadowed_value;
        s->cmp[B].actual_value = s->cmp[B].shadowed_value;
    }

    AQ_handler(opaque, -1);

    ptimer_set_count(s->timer.ptimer, (uint16_t)s->timer.tbprd.actual_value);

    for (int index = 0; index < 2; index++) {
        uint32_t cmp_value = extract32(s->cmp[index].actual_value, 16, 16);
        if ((cmp_value == 0) || (cmp_value == (uint16_t)s->timer.tbprd.actual_value)) {
            continue;
        }
        cmp_reload(opaque, index);
    }

    ptimer_run(s->timer.ptimer, 1);
}

/*
    ***ОПИСАНИЕ РАБОТЫ СЧЕТЧИКА В РЕЖИМЕ СИНХРОНИЗАЦИИ***
    В железе, если мы оставляем PHSDIR == 0 и TBPHS == 0 (и все остальные регистры
    сконфигурированны соотвественно под запуск счетчика), то в TBCTR должно
    загрузится максимальное значение счетчика и начать счет от него вниз
    (как будто, это нештатная ситуация и она нигде не описана).
    На данный момент при PHSDIR == 0 и TBPHS == 0 (и всех остальных регистрах
    сконфигурированных соотвественно под запуск счетчика) начинается счет вверх
    от нуля(собственно по логике устройства)

    Подумать, нужно ли будет это потом исправить. В остальном, режим по
    синхронизации и по обычному запуску работает исправно.
*/
static void TBCTL_handler(void *opaque) {
    EHRPWMState *s = (EHRPWMState *)opaque;

    switch(s->timer.tbctl.ctrmode) {
        case TBCTL_MODE_COUNT_UP:
            //счет вверх
            s->timer.tbsts.ctrdir = 1;
            break;
        case TBCTL_MODE_COUNT_DOWN:
            //счет вниз
            s->timer.tbsts.ctrdir = 0;
            break;
        case TBCTL_MODE_COUNT_UP_DOWN:
            //счет вверх-вниз
            if (!s->timer.tbsts.synci) { //подумать над этим позже
                s->timer.tbsts.ctrdir = 1;
            }
            break;
        case TBCTL_MODE_COUNT_STOPPED:
            //счет остановлен
            break;
    }

    //Отвечает за то, выдается ли выходной синхроимпульс, или нет
    switch(s->timer.tbctl.syncosel) { //will be useful later
        case 0:
            //EPWMxSYNC;
            break;
        case 1:
            //TBCTR=0;
            break;
        case 2:
            //TBCTR=CMPB
            break;
        case 3:
            //Запрет на выдачу синхроимпульса.
            break;
    }

    uint32_t clkdiv_value = (1 << s->timer.tbctl.clkdiv);
    uint32_t hspclkdiv_value = 
               (!s->timer.tbctl.hspclkdiv) ? 1 : (2 * s->timer.tbctl.hspclkdiv);
    
    ptimer_set_period_from_clock(s->timer.ptimer, s->pclk,
                 clkdiv_value * hspclkdiv_value);

    if (s->timer.tbctl.phsen) {
        /*
            SWFSYNC is valid (operates) only when EPWMxSYNCI is selected by SYNCOSEL = 00
            ИЗ АНГЛ. ДОКУМЕНТАЦИИ
        */ 
        if (s->timer.tbctl.swfsync/* && !s->timer.tbctl.syncosel*/) {
            if (s->timer.tbctl.ctrmode == TBCTL_MODE_COUNT_UP_DOWN) {
                s->timer.tbsts.ctrdir = s->timer.tbctl.phsdir;
            }
            /*
                костыль откровенный. посмотреть попозже, что с ним можно сделать
                Может привести к откровенно неприятным последствиям
                Обратить внимание на эту часть, при возникновении проблем с
                сигналами
            */
            if ((extract32(s->timer.tbphs, 16, 16) == 0) &&
                (s->timer.tbsts.ctrdir == TBSTS_MODE_COUNT_DOWN)) {
                s->timer.tbctr = (uint16_t)s->timer.tbprd.actual_value;
            } else {
                s->timer.tbctr = extract32(s->timer.tbphs, 16, 16);
            }
            set_count(opaque, s->timer.tbctr);
            s->timer.tbsts.synci = 1;
            s->timer.tbctl.swfsync = 0;
        }
    }

    if ((s->timer.tbctl.ctrmode == 0x3) || ((uint16_t)s->timer.tbprd.actual_value == 0)) {
        return;
    }

    for (int index = 0; index < 2; index++) {
        uint32_t cmp_value = extract32(s->cmp[index].actual_value, 16, 16);
        if ((cmp_value == 0) ||
            (cmp_value == (uint16_t)s->timer.tbprd.actual_value)) {
            continue;
        }
        switch (s->timer.tbsts.ctrdir) {
        case TBSTS_MODE_COUNT_UP:
            if (get_count(opaque) > cmp_value) {
                continue;
            }
            break;
        case TBSTS_MODE_COUNT_DOWN:
            if (get_count(opaque) < cmp_value) {
                continue;
            }
            break;
        }
        cmp_reload(opaque, index);
    }

    ptimer_run(s->timer.ptimer, 1);
}

static uint64_t EHRPWM_read(void *opaque, hwaddr offset,
        unsigned size)
{
    EHRPWMState *s = (EHRPWMState *)opaque;
    uint32_t value = 0;

    switch (offset) {
    //таймер
    case REG_TBCTL:
        value = s->timer.tbctl.reg_value;
        break;

    case REG_TBSTS:
        value = s->timer.tbsts.reg_value;
        break;

    case REG_TBPHS:
        value = s->timer.tbphs;
        break;

    case REG_TBCTR:
        s->timer.tbctr = get_count(opaque);
        value = s->timer.tbctr;
        break;

    case REG_TBPRD:
        value = s->timer.tbprd.actual_value;
        break;

    case REG_INTCLR:
        value = s->intclr;
        break;

    //компаратор
    case REG_CMPA:
        value = s->cmp[A].actual_value;
        break;
    case REG_CMPB:
        value = s->cmp[B].actual_value;
        break;

    //обработчик событий
    case REG_AQCTLA:
        value = s->aqctl[A];
        break;
    case REG_AQCTLB:
        value = s->aqctl[B];
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "EHRPWM: bad write offset " HWADDR_FMT_plx,
                      offset);
        break;
    }
    return value;
}

static void EHRPWM_write(void *opaque, hwaddr offset,
        uint64_t value, unsigned size)
{
    EHRPWMState *s = (EHRPWMState *)opaque;
    uint8_t index;

    switch (offset) {
    case REG_TBCTL:
        /*
            Выставлять бит SWFSYNC этого регистра валидно только когда
            биты SYNCOSEL этого же регистра = 00, т.е. находятся в режиме
            EPWMxSYNCI сигнала
        */ 
        s->timer.tbctl.reg_value = value;
        ptimer_transaction_begin(s->timer.ptimer);
        TBCTL_handler(opaque);
        ptimer_transaction_commit(s->timer.ptimer);
        break;

    case REG_TBSTS: //RO Статусный регистр: достижения максимального значения, Статус синхронизации, Текущее направление счёта таймера
        break;

    case REG_TBPHS: //начальная фазa Таймера
        s->timer.tbphs = value;
        break;

    case REG_TBCTR: //текущее значение счетчика таймера
        s->timer.tbctr = value;
        ptimer_transaction_begin(s->timer.ptimer);
        set_count(opaque, s->timer.tbctr);
        ptimer_transaction_commit(s->timer.ptimer);
        break;

    case REG_TBPRD: //максимальное значение счета таймера [0-15]
        s->timer.tbprd.shadowed_value = value;
        ptimer_transaction_begin(s->timer.ptimer);
        if (s->timer.tbctl.prdld || !ptimer_get_count(s->timer.ptimer)) { // || s->timer.tbctl.ctrmode == 0x3 
            s->timer.tbprd.actual_value = s->timer.tbprd.shadowed_value;
            ptimer_set_limit(s->timer.ptimer,
                             extract32(s->timer.tbprd.actual_value, 0, 16), 1);
            set_count(opaque, s->timer.tbctr);
        }
        TBCTL_handler(opaque);
        ptimer_transaction_commit(s->timer.ptimer);
        break;

    case REG_INTCLR:
        s->intclr &= ~value;
        EHRPWM_irq_update(s);
        break;

    /*
        Следующие регистры не рассматривают абсолютно все сценарии ШИМа
        и рассчитаны только на существующие драйвера. Если драйвера будут
        изменены, то и поведение этих регистров и связанных с ними элементов,
        нужно будет дописывать
    */
    //компаратор
    case REG_CMPA:      //объединить эти два варианта
    case REG_CMPB:
        index = (offset == REG_CMPA) ? A : B;
        s->cmp[index].shadowed_value = value;
        if (!ptimer_get_count(s->timer.ptimer)) {
            s->cmp[index].actual_value = s->cmp[index].shadowed_value;
        }
        break;
    //обработчик событий
    case REG_AQCTLA:
        s->aqctl[A] = value;
        break;
    case REG_AQCTLB:
        s->aqctl[B] = value;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "EHRPWM: bad write offset " HWADDR_FMT_plx,
                      offset);
        break;
    }
}

static void EHRPWM_reset(DeviceState *d)
{
    EHRPWMState *s = EHRPWM(d);
    
    /*
        В официальной доке написано, что по умолчанию счет отключен, т.е. биты
        CTRMODE регистра TBCTL выставлены в 011
    */
    s->timer.tbctl.reg_value = 0;
    s->timer.tbsts.reg_value = 0;
    s->timer.tbphs = 0;
    s->timer.tbctr = 0;
    s->timer.tbprd.shadowed_value = 0;
    s->timer.tbprd.actual_value = 0;

    s->cmpctl = 0;
    s->cmp[A].shadowed_value = 0;
    s->cmp[A].actual_value = 0;
    s->cmp[B].shadowed_value = 0;
    s->cmp[B].actual_value = 0;
    s->aqctl[A] = 0;
    s->aqctl[B] = 0;
    s->aqsfrc = 0;
    s->aqcsfrc = 0;
    s->dbctl = 0;
    s->dbred = 0;
    s->dbfed = 0;
    s->tzsel = 0;
    s->tzctl = 0;
    s->tzeint = 0;
    s->tzflg = 0;
    s->tzclr = 0;
    s->tzfrc = 0;
    s->etsel = 0;
    s->etps = 0;
    s->etflg = 0;
    s->etclr = 0;
    s->etfrc = 0;
    s->pcctl = 0;
    s->fwdth = 0;
    s->tzintclr = 0;
    s->intclr = 0;

    ptimer_transaction_begin(s->timer.ptimer);
    TBCTL_handler(d);
    ptimer_stop(s->timer.ptimer);
    ptimer_transaction_commit(s->timer.ptimer);
}

static const MemoryRegionOps EHRPWM_ops = {
    .read = EHRPWM_read,
    .write = EHRPWM_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void EHRPWM_clk_update(void *opaque, ClockEvent event) //To change later
{
    EHRPWMState *s = EHRPWM(opaque);

    ptimer_transaction_begin(s->timer.ptimer);
    ptimer_set_period_from_clock(s->timer.ptimer, s->pclk, 1);
    ptimer_transaction_commit(s->timer.ptimer);
}

static void EHRPWM_init(Object *obj)
{
    EHRPWMState *s = EHRPWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, obj, &EHRPWM_ops, s,
                          "EHRPWM", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    s->pclk = qdev_init_clock_in(DEVICE(s), "pclk",
                                 EHRPWM_clk_update, s, ClockUpdate);
}

static void EHRPWM_realize(DeviceState *dev, Error **errp)
{
    EHRPWMState *s = EHRPWM(dev);

    if (!clock_has_source(s->pclk)) {
        error_setg(errp, "EHRPWM: clk clock must be connected");
        return;
    }

    s->timer.ptimer = ptimer_init(EHRPWM_tick,
                                     s,
                                     PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);

    s->cmp_timer[A] = timer_new_ns(QEMU_CLOCK_VIRTUAL, cmp_tick_A, s);
    s->cmp_timer[B] = timer_new_ns(QEMU_CLOCK_VIRTUAL, cmp_tick_B, s);

    ptimer_transaction_begin(s->timer.ptimer);
    ptimer_set_period_from_clock(s->timer.ptimer, s->pclk, 1);
    ptimer_transaction_commit(s->timer.ptimer);
}

#if 0
static void EHRPWM_finalize(Object *obj) //подуматьЮ пригодится ли это потом
{
    EHRPWMState *s = EHRPWM(obj);
    //int i;

    ptimer_free(s->timer.ptimer);
}
#endif

static void EHRPWM_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = EHRPWM_realize;
    dc->reset = EHRPWM_reset;
}

static const TypeInfo EHRPWM_info = {
    .name          = TYPE_EHRPWM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(EHRPWMState),
    .instance_init = EHRPWM_init,
#if 0
    .instance_finalize = EHRPWM_finalize,
#endif
    .class_init    = EHRPWM_class_init,
};

static void EHRPWM_register_types(void)
{
    type_register_static(&EHRPWM_info);
}

type_init(EHRPWM_register_types)
