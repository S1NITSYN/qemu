#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "hw/char/serial.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "chardev/char-serial.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "qemu/error-report.h"
#include "trace.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"

#include "hw/comm-port/commport.h"


//////
#define     REG_RD_MAIN_COUNTER     0x0
#define     REG_RD_CURR_ADDRESS     0x4
#define     REG_RD_BIAS             0x8
#define     REG_RD_ROW_COUNTER      0xC
#define     REG_RD_ADDRESSING_MODE  0x10
#define     REG_RD_CSR              0x14
#define     REG_RD_INERRUPT         0x18
#define     REG_RD_STATE            0x1C

#define     REG_WR_MAIN_COUNTER     0x20
#define     REG_WR_CURR_ADDRESS     0x24
#define     REG_WR_BIAS             0x28
#define     REG_WR_ROW_COUNTER      0x2C
#define     REG_WR_ADDRESSING_MODE  0x30
#define     REG_WR_CSR              0x34
#define     REG_WR_INERRUPT         0x38
#define     REG_WR_STATE            0x3C

#define     REG_OPTIONAL_HC         0x40
#define     REG_OPTIONAL_PHY        0x44
#define     REG_OPTIONAL_ERR1_C     0x68
#define     REG_OPTIONAL_ERR2_C     0x6C



//Registers
#define COMM_REG_ID                0x0
#define COMM_REG_TX                0x4
#define COMM_REG_RX                0x8
#define COMM_REG_BAUD              0x508
#define COMM_PACKET_LEN            64      //bits
#define COMM_MEM_WR_LEN            8       //bits
#define COMM_CHAR_LEN              1       //bits


/*FOR REG_CSR*/
#define COMM_CTRL_EN               (1 << 0)
#define COMM_CTRL_CPL              (1 << 1)
#define COMM_CTRL_ES               (1 << 2)
#define COMM_CTRL_CLR              (1 << 3)
#define COMM_FULL_CTRL_MASK        0xF

/*FOR BUF*/
#define COMM_STATUS_CURR_SIZE      (0xFFFF << 4)
#define COMM_STATUS_TX_EMPTY       (1 << 0)
#define COMM_STATUS_TX_FULL        (1 << 1)
#define COMM_STATUS_RX_NOT_EMPTY   (1 << 2)
#define COMM_STATUS_RX_FULL        (1 << 3)

/*FOR CONTROLLER STATE*/
/*счётчик доступных данных*/
#define GET_ADC(InternalState) (InternalState & 0x7)
#define SET_ADC(value)         value

/*счётчик незавершённых транзакций*/
#define GET_UTC(InternalState) (InternalState >> 18) & 0x7
#define SET_UTC(value)         (value << 18)

/*При приостановке передающей части сумма RD_MainCounter + ARC показывает реальное
 количество "недоотправленных" на коммуникационную шину данных;*/
#define GET_ARC(InternalState) (InternalState & 0x1F)
#define SET_ARC(value)         value
/*текущее состояние конечного автомата*/
#define GET_FSM(InternalState) (InternalState >> 24) & 0x1F
#define SET_FSM(value)         (value << 24)
#define SET_STATUS_NULL        ~(0x1F)

#define FSM_idle                    0x0
#define FSM_complete                0x2
#define FSM_data_miss               0x6

//especially write
#define FSM_receive_write           0x1
#define FSM_uncomplete_write        0x3

//especially read
#define FSM_read_send               0x1
#define FSM_send_only               0x3
//#define COMM_REG_STATUS        0x01

#define READ_CONDITION


static uint64_t comm_read(void *opaque, hwaddr addr, unsigned size)
{
    comm_state *s = opaque;
    uint64_t incoming_word;

    switch (addr) {
    case REG_RD_MAIN_COUNTER:
        return s->comm_rd_main_counter;

    case REG_RD_CURR_ADDRESS:
        printf("point has been reached a bit \n\n");
        if (s->comm_rd_main_counter) {
            printf("point has been reached \n\n");
            incoming_word = (uint64_t)s->comm_rx; //may get broken
            memmove(s->comm_rx, s->comm_rx + 8, s->comm_rd_main_counter - 1); //second arg can be s->comm_rx[1]
            s->comm_rd_main_counter--;
            qemu_chr_fe_accept_input(&s->comm_chr);
            //irq later
            return incoming_word;
        }
        return s->comm_rd_address;

    case REG_RD_BIAS:
        return s->comm_rd_bias;

    case REG_RD_ROW_COUNTER:
        return s->comm_rd_row_counter;

    case REG_RD_ADDRESSING_MODE:
        return s->comm_rd_addressing_mode;

    case REG_RD_CSR:
        return s->comm_rd_CSR;

    case REG_RD_INERRUPT:
        return s->comm_rd_interrupt_mask;

    case REG_RD_STATE:
        return s->comm_rd_status;

    case REG_WR_MAIN_COUNTER:
        return s->comm_wr_main_counter;

    case REG_WR_CURR_ADDRESS:
        return s->comm_wr_address;

    case REG_WR_BIAS:
        return s->comm_wr_bias;

    case REG_WR_ROW_COUNTER:
        return s->comm_wr_row_counter;

    case REG_WR_ADDRESSING_MODE:
        return s->comm_wr_addressing_mode;
    case REG_WR_CSR:
        return s->comm_wr_CSR;

    case REG_WR_INERRUPT:
        return s->comm_wr_interrupt_mask;

    case REG_WR_STATE:
        return s->comm_wr_status;

    /*case REG_OPTIONAL_HC:
        val = s->comm_rd_row_counter;

    case REG_OPTIONAL_PHY:
        val = s->comm_rd_row_counter;

    case REG_OPTIONAL_ERR2_C:
        val = s->comm_rd_row_counter;

    case REG_OPTIONAL_ERR1_C:

        val = s->comm_rd_row_counter;*/

    default: return 0;
    }
    return 0;
}


static void comm_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    comm_state *s = COMM(opaque);
    uint8_t char_value = (uint8_t)val;
    switch (addr) {
    case REG_RD_MAIN_COUNTER:
        s->comm_rd_main_counter = val;
        break;

    case REG_RD_CURR_ADDRESS:
        s->comm_rd_address = val;
        break;

    case REG_RD_BIAS:
        s->comm_rd_bias = val;
        break;

    case REG_RD_ROW_COUNTER:
        s->comm_rd_row_counter = val;
        break;

    case REG_RD_ADDRESSING_MODE:
        s->comm_rd_addressing_mode = val;
        break;

    case REG_RD_CSR:
        s->comm_rd_CSR = val;
        break;

    case REG_RD_INERRUPT:
        s->comm_rd_interrupt_mask = val;
        break;


    case REG_WR_MAIN_COUNTER:
        s->comm_wr_main_counter = val;
        break;

    case REG_WR_CURR_ADDRESS:
        qemu_chr_fe_write(&s->comm_chr, &char_value, COMM_CHAR_LEN);
        //irq
        break;

    case REG_WR_BIAS:
        s->comm_wr_bias = val;
        break;

    case REG_WR_ROW_COUNTER:
        s->comm_wr_row_counter = val;
        break;

    case REG_WR_ADDRESSING_MODE:
        s->comm_wr_addressing_mode = val;
        break;

    case REG_WR_CSR:
        s->comm_wr_CSR = val;
        break;

    case REG_WR_INERRUPT:
        s->comm_wr_interrupt_mask = val;
        break;


    /*case REG_OPTIONAL_HC:
        val = s->comm_rd_row_counter;
        break;

    case REG_OPTIONAL_PHY:
        val = s->comm_rd_row_counter;
        break;

    case REG_OPTIONAL_ERR2_C:
        val = s->comm_rd_row_counter;
        break;

    case REG_OPTIONAL_ERR1_C:

        val = s->comm_rd_row_counter;
        break;*/

    default: break;
    }
}
///qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK,
//                              &break_enable);

static const MemoryRegionOps comm_ops = {
    .read = comm_read,
    .write = comm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    //тут добавить колбеки для serial
    /*.valid {
        .min_access_size = 4, //cкорее всего, сколько мининмум данных может съесть cpu
        .max_access_size = 4  //а тут наоборот
    }*/
};

static void comm_reset(DeviceState *dev)
{
    comm_state *s = COMM(dev);

    s->comm_rx_counter = 0;
    s->comm_rx_max_size = 1000; //hardcode for now
    //s->comm_baudbase = 115200;
    s->comm_data_reg = 0;
    s->comm_rd_ctrl = 0;
    s->comm_wr_ctrl = 0;
    s->comm_rd_status = 0;
    s->comm_wr_status = 0;
    s->comm_rd_buff_status = 0;
    s->comm_wr_buff_status = 0;
    s->comm_wr_address = 0;
    s->comm_rd_address = 0;
    s->comm_rd_main_counter = 0;
    s->comm_wr_main_counter = 0;
    s->comm_rd_bias = 0;
    s->comm_wr_bias = 0;
    s->comm_wr_row_counter = 0;
    s->comm_rd_row_counter = 0;
    s->comm_rd_CSR = 0;
    s->comm_wr_CSR = 0;
    s->comm_rd_interrupt_mask = 0;
    s->comm_wr_interrupt_mask = 0;
    s->comm_rd_internal_state = 0;
    s->comm_wr_internal_state = 0;
    //s->comm_status = 0x0;

    //qemu_set_irq(s->irq, 0);
}

static int comm_can_receive(void *opaque)
{
    comm_state *s = opaque;
    printf("point comm_can_receive \n\n");

    return ((s->comm_rx_counter != 64) || s->comm_rd_main_counter != 8) ?
                                                                       COMM_CHAR_LEN : 0;//!(s->comm_status & comm_STATUS_RX_NOT_EMPTY); //define
} //about 8 - comm_rx can contain max 8 64 bit words


static void comm_receive(void *opaque, const uint8_t *data_char, int size)
{
    printf("point comm_receive \n\n");
    comm_state *s = opaque;
    memmove(s->comm_rx + s->comm_rx_counter, data_char, COMM_CHAR_LEN);
    s->comm_rx_counter += COMM_CHAR_LEN;
    if (!(s->comm_rx_counter % COMM_MEM_WR_LEN))
        s->comm_rd_main_counter++;
}


static Property comm_properties[] = {
    DEFINE_PROP_CHR("CommChardev", comm_state, comm_chr),
    //DEFINE_PROP_UINT32("baude", comm_state, comm_baudbase, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void comm_event(void *opaque, QEMUChrEvent event)
{

}

static void comm_realize(DeviceState *dev, Error **errp)
{
    comm_state *s = COMM(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    //sysbus_init_irq();
    memory_region_init_io(&s->iomem, OBJECT(dev), &comm_ops, s, "comm", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    qemu_chr_fe_set_handlers(&s->comm_chr, comm_can_receive,
                             comm_receive, comm_event, NULL, s, NULL, true);/////////////
}

static void comm_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->reset = comm_reset;
    dc->realize = comm_realize;
    dc->desc = "scibidi dop dop yes yes";
    device_class_set_props(dc, comm_properties);
}

static const TypeInfo comm_info = {
    .name = TYPE_COMM,
    .parent = TYPE_SYS_BUS_DEVICE, //TYPE_DEVICE
    .instance_size = sizeof(comm_state),
    .class_init = comm_class_init,
};


static void comm_register_types(void)
{
    type_register_static(&comm_info);
}

type_init(comm_register_types)
