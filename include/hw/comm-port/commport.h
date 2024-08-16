#ifndef COMM_H
#define COMM_H
#include "hw/sysbus.h"
#include "chardev/char-fe.h"


struct comm_state
{
    SysBusDevice parent;
    MemoryRegion iomem;

    AddressSpace *addr_space;

    //uint64_t chip_id;
    uint32_t comm_data_reg;
    uint32_t comm_rx_max_size;
    uint8_t comm_rx_counter;
    uint8_t comm_rx[64]; //still dont sure in 64. can be more bcs on physic was translated 12 bytes(96 bits > 64) and more
    //uint32_t comm_baudbase; //могу задавать произвольно
    uint32_t comm_ctrl;
    uint32_t comm_status; //GET_ARC - Счётчик активных запросов
    uint32_t comm_buff_status; //for
    uint32_t comm_address; //могу задавать произвольно
    uint32_t comm_main_counter; //главный счетчик
    uint32_t comm_bias;
    uint32_t comm_row_counter; //for addr generator
    uint32_t comm_addressing_mode;
    uint32_t comm_CSR; //управления состоянием контроллера; биты состояния EN/ES/CLR/CPL
    uint32_t comm_interrupt_mask;
    uint32_t comm_internal_state; //показывает внутреннее состояние передающей части контроллера; состояния Idle/readsend/sendonly/datamiss/complete
    uint32_t comm_current_operation_type;

    CharBackend comm_chr;
};

typedef struct comm_state comm_state;

#define TYPE_COMM "comm"
#define COMM(obj) OBJECT_CHECK(comm_state, (obj), TYPE_COMM)

void comm_change_address_space(comm_state *s, AddressSpace *addr_space, Error **errp);

#endif


   /* uint32_t comm_data_reg;
    uint32_t comm_rx_max_size;
    uint8_t comm_rx_counter;
    uint8_t comm_rx[64]; //still dont sure in 64. can be more bcs on physic was translated 12 bytes(96 bits > 64) and more
    //uint32_t comm_baudbase; //могу задавать произвольно
    uint32_t comm_rd_ctrl;
    uint32_t comm_wr_ctrl;
    uint32_t comm_rd_status; //GET_ARC - Счётчик активных запросов
    uint32_t comm_wr_status;
    uint32_t comm_rd_buff_status; //for
    uint32_t comm_wr_buff_status;
    uint32_t comm_rd_address; //могу задавать произвольно
    uint32_t comm_wr_address; //могу задавать произвольно
    uint32_t comm_rd_main_counter; //главный счетчик
    uint32_t comm_wr_main_counter;
    uint32_t comm_rd_bias;
    uint32_t comm_wr_bias; //xz later
    uint32_t comm_rd_row_counter; //for addr generator
    uint32_t comm_wr_row_counter;
    uint32_t comm_rd_addressing_mode;
    uint32_t comm_wr_addressing_mode;
    uint32_t comm_rd_CSR; //управления состоянием контроллера; биты состояния EN/ES/CLR/CPL
    uint32_t comm_wr_CSR;
    uint32_t comm_rd_interrupt_mask;
    uint32_t comm_wr_interrupt_mask;
    uint32_t comm_rd_internal_state; //показывает внутреннее состояние передающей части контроллера; состояния Idle/readsend/sendonly/datamiss/complete
    uint32_t comm_wr_internal_state;*/