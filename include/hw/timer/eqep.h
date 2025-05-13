#ifndef HW_TIMER_EQEP_H
#define HW_TIMER_EQEP_H

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
        uint32_t : 5;
        uint32_t qsp : 1;
        uint32_t qip : 1;
        uint32_t qbp : 1;
        uint32_t qap : 1;
        uint32_t igate : 1;
        uint32_t swap : 1;
        uint32_t xcr : 1;
        uint32_t spsel : 1;
        uint32_t soen : 1;
        uint32_t qsrc : 2;
        uint32_t : 16;
    };
    uint32_t reg_value;
} qdecctl_t;

typedef union {
    struct {
        uint32_t upps : 4;
        uint32_t ccps : 3;
        uint32_t selevnt : 1;
        uint32_t : 7;
        uint32_t cen : 1;
        uint32_t : 16;
    };
    uint32_t reg_value;
} qcapctl_t;

typedef union {
    struct {
        uint32_t pcef : 1;
        uint32_t fimf : 1;
        uint32_t cdef : 1;
        uint32_t coef : 1;
        uint32_t qdlf : 1;
        uint32_t qdf : 1;
        uint32_t fidf : 1;
        uint32_t upevnt : 1;
        uint32_t : 24;
    };
    uint32_t reg_value;
} qepsts_t;

#define TYPE_EQEP "EQEP"
OBJECT_DECLARE_SIMPLE_TYPE(EQEPState, EQEP)

struct EQEPState{
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    Clock *pclk;
    ptimer_state *ptimer;

    uint32_t qposcnt;
    uint32_t qposinit;
    uint32_t qposmax;
    uint32_t qposcmp;
    uint32_t qposilat;
    uint32_t qposslat;
    uint32_t qposlat;
    uint32_t qutmr;
    uint32_t quprd;
    uint32_t qwdtmr;
    uint32_t qwdprd;
    qdecctl_t qdecctl; // Регистр управления входами
    uint32_t qepctl;
    qcapctl_t qcapctl; // Регистр захвата
    uint32_t qposctl;
    uint32_t qeint;
    uint32_t qflg;
    uint32_t qclr;
    uint32_t qfrc;
    qepsts_t qepsts;
    uint32_t qctmr; // значение таймера времени для измерений
    uint32_t qcprd; // сохраненное значение таймера времени при последнем измерении
    uint32_t qctmrlat;
    uint32_t qcprdlat;
    uint32_t intclr;


    qemu_irq irq;
    uint8_t QEPx[2];
    /*
        Компаратор текущей позиции сравнивает значение счётчика позиции
        (QPOSCNT) (см. рисунок 3.5.2) и значение регистра сравнения (QPOSCMP). При
        равенстве значений этих регистров формируется сигнал синхронизации, который
        может быть направлен на один из выводов: индексный вывод EQEPI или вывод
        стробирования QEPS. Регистр QDECCTL[SPSEL] определяет, на какой именно
        вывод будет направлен сигнал синхронизации, а регистр QDECCTL[SOEN]
        разрешает этому выводу работать на выход.

        Для вывода sync
    */
    qemu_irq sync;
};

#endif //HW_TIMER_EQEP_H
