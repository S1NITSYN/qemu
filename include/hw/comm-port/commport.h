#ifndef COMM_H
#define COMM_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"

struct CommState {
    SysBusDevice parent;
    MemoryRegion iomem;

    AddressSpace *addr_space;

    uint32_t comm_receive_main_counter;  // главный счетчик
    uint32_t comm_receive_address;  // могу задавать произвольно
    uint32_t comm_receive_bias;
    uint32_t comm_receive_row_counter;  // for addr generator
    uint32_t comm_receive_addressing_mode;
    uint32_t comm_receive_CSR;  // управления состоянием контроллера; биты
                                // состояния EN/ES/CLR/CPL
    uint32_t comm_receive_interrupt_mask;
    uint32_t comm_receive_internal_state;  // GET_ARC - Счётчик активных
                                           // запросов
    uint32_t comm_transmit_main_counter;  // главный счетчик
    uint32_t comm_transmit_address;  // могу задавать произвольно
    uint32_t comm_transmit_bias;
    uint32_t comm_transmit_row_counter;  // for addr generator
    uint32_t comm_transmit_addressing_mode;
    uint32_t comm_transmit_CSR;  // управления состоянием контроллера; биты
                                 // состояния EN/ES/CLR/CPL
    uint32_t comm_transmit_interrupt_mask;
    uint32_t comm_transmit_internal_state;
    uint32_t comm_optional_hc;
    uint32_t comm_optional_err1_c;
    uint32_t comm_optional_err2_c;
    uint8_t registers_mode;

    CharBackend comm_chr;
    qemu_irq irq0;
    qemu_irq irq1;
};

typedef struct CommState CommState;
typedef uint32_t nmc_byte_t;

#define TYPE_COMM "comm"
#define COMM(obj) OBJECT_CHECK(CommState, (obj), TYPE_COMM)

void comm_change_address_space(CommState *s, AddressSpace *addr_space,
                               Error **errp);

#endif /* COMM_H */