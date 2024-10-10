#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/irq.h"
#include "sysemu/dma.h"
#include "hw/sysbus.h"
#include "chardev/char-serial.h"

#include "hw/comm-port/commport.h"

#define REG_TRANSMIT_MAIN_COUNTER 0x0
#define REG_TRANSMIT_CURR_ADDRESS 0x2
#define REG_TRANSMIT_BIAS 0x4
#define REG_TRANSMIT_ROW_COUNTER 0x6
#define REG_TRANSMIT_ADDRESSING_MODE 0x8
#define REG_TRANSMIT_CSR 0xA
#define REG_TRANSMIT_INTERRUPT 0xC
#define REG_TRANSMIT_STATE 0xE

#define REG_RECEIVE_MAIN_COUNTER 0x10
#define REG_RECEIVE_CURR_ADDRESS 0x12
#define REG_RECEIVE_BIAS 0x14
#define REG_RECEIVE_ROW_COUNTER 0x16
#define REG_RECEIVE_ADDRESSING_MODE 0x18
#define REG_RECEIVE_CSR 0x1A
#define REG_RECEIVE_INTERRUPT 0x1C
#define REG_RECEIVE_STATE 0x1E

#define REG_OPTIONAL_HC 0x20
#define REG_OPTIONAL_ERR1_C 0x34
#define REG_OPTIONAL_ERR2_C 0x36

#define PACKET_LEN_STANDART 8
#define PACKET_LEN_WITH_HAMMING 11

#define COMM_CTRL_EN 0x1
#define COMM_CTRL_CPL 0x2
#define COMM_CTRL_ES 0x4
#define COMM_CTRL_CLR 0x8

#define COMM_INTERNAL_STATE_IDLE 0x0
#define COMM_INTERNAL_STATE_COMPLETE (0x2 << 23)
#define COMM_INTERNAL_STATE_READ_SEND (0x1 << 23)
#define COMM_INTERNAL_STATE_RECEIVE_WRITE (0x1 << 23)

#define DROP_LAST_3BITS(addr) (addr & ~7ull)

#define PPC_MODE 1
#define NMC_MODE 2

static void comm_update_irq(comm_state *s)
{
    if (s->comm_transmit_CSR & (COMM_CTRL_CPL | COMM_CTRL_ES)) {
        qemu_irq_raise(s->irq0);
    } else
        qemu_irq_lower(s->irq0);
    if (s->comm_receive_CSR & (COMM_CTRL_CPL | COMM_CTRL_ES)) {
        qemu_irq_raise(s->irq1);
    } else
        qemu_irq_lower(s->irq1);
}

static void comm_tx(comm_state *s)
{
    uint8_t *buffer;
    uint8_t size =
        (s->comm_optional_hc) ? PACKET_LEN_WITH_HAMMING : PACKET_LEN_STANDART;

    buffer = malloc(size * sizeof(uint8_t));
    uint8_t dma_read_result;
    printf("write has been reached fully\n\n");
    for (int i = 0; i < s->comm_transmit_main_counter; i++) {
        dma_read_result =
            dma_memory_read(s->addr_space, s->comm_transmit_address, buffer,
                            PACKET_LEN_STANDART, MEMTXATTRS_UNSPECIFIED);

        if (dma_read_result != MEMTX_OK) {
            s->comm_transmit_CSR = COMM_CTRL_ES;
            s->comm_transmit_internal_state = COMM_INTERNAL_STATE_IDLE;
            comm_update_irq(s);
            return;
        }

        qemu_chr_fe_write_all(&s->comm_chr, buffer, size);
        s->comm_transmit_address += PACKET_LEN_STANDART;
    }

    s->comm_transmit_main_counter = 0;
    if ((s->comm_transmit_CSR == COMM_CTRL_EN) &&
        !s->comm_transmit_main_counter) {
        s->comm_transmit_CSR = COMM_CTRL_CPL;
        s->comm_transmit_internal_state = COMM_INTERNAL_STATE_COMPLETE;
        comm_update_irq(s);
    }
    printf("%s %x\n\n", "MAINCOUNTER", s->comm_transmit_main_counter);
    printf("%s %lx\n\n", "ADDRESS",
           (s->registers_mode == NMC_MODE)
               ? (s->comm_transmit_address / sizeof(nmc_byte_t))
               : s->comm_transmit_address);
    printf("%s %x\n\n", "CSR", s->comm_transmit_CSR);
}

static void comm_rx(comm_state *s, const uint8_t *buffer)
{
    uint8_t dma_write_result;
    printf("read has been reached fully\n\n");
    dma_write_result =
        dma_memory_write(s->addr_space, s->comm_receive_address, buffer,
                         PACKET_LEN_STANDART, MEMTXATTRS_UNSPECIFIED);

    if (dma_write_result != MEMTX_OK) {
        s->comm_receive_CSR = COMM_CTRL_ES;
        s->comm_receive_internal_state = COMM_INTERNAL_STATE_IDLE;
        comm_update_irq(s);
        return;
    }

    s->comm_receive_address += PACKET_LEN_STANDART;
    s->comm_receive_main_counter--;
    if ((s->comm_receive_CSR == COMM_CTRL_EN) &&
        !s->comm_receive_main_counter) {
        s->comm_receive_CSR = COMM_CTRL_CPL;
        s->comm_receive_internal_state = COMM_INTERNAL_STATE_COMPLETE;
        comm_update_irq(s);
    }
    printf("%s %x\n\n", "MAINCOUNTER", s->comm_receive_main_counter);
    printf("%s %lx\n\n", "ADDRESS",
           (s->registers_mode == NMC_MODE)
               ? (s->comm_receive_address / sizeof(nmc_byte_t))
               : s->comm_receive_address);
    printf("%s %x\n\n", "CSR", s->comm_receive_CSR);
}

static uint64_t comm_read(void *opaque, hwaddr addr, unsigned size)
{
    comm_state *s = opaque;
    uint64_t val = 0;
    addr >>= s->registers_mode;

    switch (addr) {
    case REG_TRANSMIT_MAIN_COUNTER:
        val = s->comm_transmit_main_counter;
        printf("%s\n", "READ FROM MAIN_COUNTER");

        break;
    case REG_RECEIVE_MAIN_COUNTER:
        val = s->comm_receive_main_counter;
        printf("%s\n", "READ FROM MAIN_COUNTER");

        break;

    case REG_TRANSMIT_CURR_ADDRESS:
        val = (s->registers_mode == NMC_MODE)
                  ? (s->comm_transmit_address / sizeof(nmc_byte_t))
                  : s->comm_transmit_address;
        printf("%s\n", "READ FROM CURR_ADDRESS");

        break;
    case REG_RECEIVE_CURR_ADDRESS:
        val = (s->registers_mode == NMC_MODE)
                  ? (s->comm_receive_address / sizeof(nmc_byte_t))
                  : s->comm_receive_address;
        printf("%s\n", "READ FROM CURR_ADDRESS");

        break;

    case REG_TRANSMIT_BIAS:
        val = s->comm_transmit_bias;
        printf("%s\n", "READ FROM BIAS");

        break;
    case REG_RECEIVE_BIAS:
        val = s->comm_receive_bias;
        printf("%s\n", "READ FROM BIAS");

        break;

    case REG_TRANSMIT_ROW_COUNTER:
        val = s->comm_transmit_row_counter;
        printf("%s\n", "READ FROM ROW_COUNTER");

        break;
    case REG_RECEIVE_ROW_COUNTER:
        val = s->comm_receive_row_counter;
        printf("%s\n", "READ FROM ROW_COUNTER");

        break;

    case REG_TRANSMIT_ADDRESSING_MODE:
        val = s->comm_transmit_addressing_mode;
        printf("%s\n", "READ FROM ADDRESSING_MODE");

        break;
    case REG_RECEIVE_ADDRESSING_MODE:
        val = s->comm_receive_addressing_mode;
        printf("%s\n", "READ FROM ADDRESSING_MODE");

        break;

    case REG_TRANSMIT_CSR:
        val = s->comm_transmit_CSR;
        // printf("%lx\t%s\n", val, "READ FROM TRANSMIT CSR");
        break;
    case REG_RECEIVE_CSR:
        val = s->comm_receive_CSR;
        // printf("%lx\t%s\n", val, "READ FROM RECEIVE CSR");
        break;

    case REG_TRANSMIT_INTERRUPT:
        val = s->comm_transmit_interrupt_mask;
        printf("%s\n", "READ FROM INTERRUPT");

        break;
    case REG_RECEIVE_INTERRUPT:
        val = s->comm_receive_interrupt_mask;
        printf("%s\n", "READ FROM INTERRUPT");

        break;

    case REG_TRANSMIT_STATE:
        val = s->comm_transmit_internal_state;
        printf("%s\n", "READ FROM STATE");

        break;
    case REG_RECEIVE_STATE:
        val = s->comm_receive_internal_state;
        printf("%s\n", "READ FROM STATE");

        break;
    case REG_OPTIONAL_HC:
        val = s->comm_optional_hc;
        printf("%s\n", "READ FROM HC");

        break;
    case REG_OPTIONAL_ERR1_C:
        val = s->comm_optional_err1_c;
        printf("%s\n", "READ FROM ERR1");

        break;
    case REG_OPTIONAL_ERR2_C:
        val = s->comm_optional_err2_c;
        printf("%s\n", "READ FROM ERR2");

        break;

    default:
        break;
    }

    return val;
}

static void comm_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    comm_state *s = COMM(opaque);
    addr >>= s->registers_mode;

    switch (addr) {
    case REG_TRANSMIT_MAIN_COUNTER:
        s->comm_transmit_main_counter = val;
        printf("%s\n", "WROTE TO TRANSMIT MAIN_COUNTER");

        break;
    case REG_RECEIVE_MAIN_COUNTER:
        s->comm_receive_main_counter = val;
        printf("%s\n", "WROTE TO RECEIVEMAIN_COUNTER");

        break;

    case REG_TRANSMIT_CURR_ADDRESS:
        s->comm_transmit_address = DROP_LAST_3BITS(
            (s->registers_mode == NMC_MODE) ? (val * sizeof(nmc_byte_t)) : val);
        printf("%s\n", "WROTE TO TRANSMIT CURR_ADDRESS");

        break;
    case REG_RECEIVE_CURR_ADDRESS:
        s->comm_receive_address = DROP_LAST_3BITS(
            (s->registers_mode == NMC_MODE) ? (val * sizeof(nmc_byte_t)) : val);
        printf("%s\n", "WROTE TO RECEIVE CURR_ADDRESS");

        break;

    case REG_TRANSMIT_BIAS:
        s->comm_transmit_bias = val;
        printf("%s\n", "WROTE TO TRANSMIT BIAS");

        break;
    case REG_RECEIVE_BIAS:
        s->comm_receive_bias = val;
        printf("%s\n", "WROTE TO RECEIVE BIAS");

        break;

    case REG_TRANSMIT_ROW_COUNTER:
        s->comm_transmit_row_counter = val;
        printf("%s\n", "WROTE TO TRANSMIT ROW_COUNTER");

        break;
    case REG_RECEIVE_ROW_COUNTER:
        s->comm_receive_row_counter = val;
        printf("%s\n", "WROTE TO RECEIVE ROW_COUNTER");

        break;

    case REG_TRANSMIT_ADDRESSING_MODE:
        s->comm_transmit_addressing_mode = val;
        printf("%s\n", "WROTE TO TRANSMIT ADDRESSING_MODE");

        break;
    case REG_RECEIVE_ADDRESSING_MODE:
        s->comm_receive_addressing_mode = val;
        printf("%s\n", "WROTE TO RECEIVE ADDRESSING_MODE");

        break;

    case REG_TRANSMIT_CSR:
        printf("%s\n", "WROTE TO TRANSMIT CSR");
        s->comm_transmit_CSR = val;
        if (s->comm_transmit_CSR != COMM_CTRL_EN) {
            break;
        }
        s->comm_transmit_internal_state = COMM_INTERNAL_STATE_READ_SEND;
        comm_tx(s);

        break;
    case REG_RECEIVE_CSR:
        printf("%s\n", "WROTE TO RECEIVE CSR");
        s->comm_receive_CSR = val;
        if (s->comm_receive_CSR != COMM_CTRL_EN) {
            break;
        }
        qemu_chr_fe_accept_input(&s->comm_chr);

        break;

    case REG_TRANSMIT_INTERRUPT:
        s->comm_transmit_interrupt_mask = val;
        printf("%s\n", "WROTE TO RECEIVE INTERRUPT");

        break;
    case REG_RECEIVE_INTERRUPT:
        s->comm_receive_interrupt_mask = val;
        printf("%s\n", "WROTE TO TRANSMIT INTERRUPT");

        break;

    case REG_OPTIONAL_HC:
        s->comm_optional_hc = val;
        printf("%s\n", "WROTE TO HC");

        break;
    case REG_OPTIONAL_ERR1_C:
        s->comm_optional_err1_c = val;
        printf("%s\n", "WROTE TO ERR1");

        break;
    case REG_OPTIONAL_ERR2_C:
        s->comm_optional_err2_c = val;
        printf("%s\n", "WROTE TO ERR2");

        break;

    default:
        break;
    }
}

static const MemoryRegionOps comm_ops = {
    .read = comm_read,
    .write = comm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void comm_reset(DeviceState *dev)
{
    comm_state *s = COMM(dev);

    s->comm_receive_main_counter = 0;
    s->comm_receive_address = 0;
    s->comm_receive_bias = 0;
    s->comm_receive_row_counter = 0;
    s->comm_receive_addressing_mode = 0;
    s->comm_receive_CSR = 0;
    s->comm_receive_interrupt_mask = 0;
    s->comm_receive_internal_state = COMM_INTERNAL_STATE_IDLE;
    s->comm_transmit_main_counter = 0;
    s->comm_transmit_address = 0;
    s->comm_transmit_bias = 0;
    s->comm_transmit_row_counter = 0;
    s->comm_transmit_addressing_mode = 0;
    s->comm_transmit_CSR = 0;
    s->comm_transmit_interrupt_mask = 0;
    s->comm_transmit_internal_state = COMM_INTERNAL_STATE_IDLE;
    s->comm_optional_hc = 0;
    s->comm_optional_err1_c = 0;
    s->comm_optional_err2_c = 0;
}

static int comm_can_receive(void *opaque)
{
    comm_state *s = opaque;

    if (s->comm_receive_CSR != COMM_CTRL_EN) {
        return 0;
    }
    if (s->comm_optional_hc) {
        return PACKET_LEN_WITH_HAMMING;
    }
    printf("point comm_can_receive \n\n");

    return PACKET_LEN_STANDART;
}

static void comm_receive(void *opaque, const uint8_t *data_char, int size)
{
    comm_state *s = opaque;
    printf("point comm_receive \n\n");

    if (s->comm_receive_CSR == COMM_CTRL_ES) {
        return;
    }
    s->comm_receive_internal_state = COMM_INTERNAL_STATE_RECEIVE_WRITE;

    comm_rx(s, data_char);
    printf("ACTION \n\n");
}

void comm_change_address_space(comm_state *s, AddressSpace *addr_space,
                               Error **errp)
{
    if (object_property_get_bool(OBJECT(s), "realized", errp)) {
        error_setg(errp, "Can't change address_space of realized device\n");
    }

    s->addr_space = addr_space;
}

static Property comm_properties[] = {
    DEFINE_PROP_CHR("CommChardev", comm_state, comm_chr),
    DEFINE_PROP_UINT8("RegistersMode", comm_state, registers_mode,
                      0),  // PPC_MODE - 1, NMC_MODE - 2
    DEFINE_PROP_END_OF_LIST(),
};

static void comm_realize(DeviceState *dev, Error **errp)
{
    comm_state *s = COMM(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    memory_region_init_io(&s->iomem, OBJECT(dev), &comm_ops, s, "comm", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq0);
    sysbus_init_irq(sbd, &s->irq1);
    qemu_chr_fe_set_handlers(&s->comm_chr, comm_can_receive, comm_receive, NULL,
                             NULL, s, NULL, true);
    if (s->addr_space == NULL) {
        s->addr_space = &address_space_memory;
    }
}

static void comm_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->reset = comm_reset;
    dc->realize = comm_realize;
    dc->desc =
        "Communication port for data exchange with the NeuroMatrix "
        "architecture neuroprocessor";
    device_class_set_props(dc, comm_properties);
}

static const TypeInfo comm_info = {
    .name = TYPE_COMM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(comm_state),
    .class_init = comm_class_init,
};

static void comm_register_types(void)
{
    type_register_static(&comm_info);
}

type_init(comm_register_types)