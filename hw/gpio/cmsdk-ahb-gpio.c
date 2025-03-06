#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "qemu/units.h"
#include "hw/gpio/cmsdk-ahb-gpio.h"

#define REG_DATA		0x0
#define REG_DATAOUT		0x4
#define REG_OUTENSET	0x10
#define REG_OUTENCLR	0x14
#define REG_ALTFUNCSET	0x18
#define REG_ALTFUNCCLR	0x1c
#define REG_INTENSET	0x20
#define REG_INTENCLR	0x24
#define REG_INTTYPESET	0x28
#define REG_INTTYPECLR	0x2C
#define REG_INTPOLSET	0x30
#define REG_INTPOLCLR	0x34
#define REG_INTSTATUS	0x38
#define MASKLOWBYTE_START	0x400
#define MASKLOWBYTE_END		0x7FC
#define MASKHIGHBYTE_START	0x800
#define MASKHIGHBYTE_END	0xBFC

static void CMSDKAHB_GPIO_update_int(CMSDKAHB_GPIOState *s)
{	
	if (!s->intstatus) {
		qemu_irq_lower(s->irq);
		printf("%s\n", "dropped irq");
	} else if (s->inttype & s->intstatus) {
		qemu_irq_pulse(s->irq);
		printf("%s\n", "pulse irq");
	} else {
		qemu_irq_raise(s->irq);
		printf("%s\n", "set irq");
	}
}

static void CMSDKAHB_GPIO_set_int_line(CMSDKAHB_GPIOState *s, int line, uint8_t level)
{
	if (!extract32(s->inten, line, 1)) {
		return;
	}
	uint32_t polarity = extract32(s->intpol, line, 1);
	uint32_t type = extract32(s->inttype, line, 1);

	if ((polarity == level) && (!type || (type && (polarity != extract32(s->data, line, 1))))) {
    	s->intstatus = deposit32(s->intstatus, line, 1, 1);
	} else {
		s->intstatus = deposit32(s->intstatus, line, 1, 0);
	}
}

static void CMSDKAHB_GPIO_set(void *opaque, int line, int level)
{
    CMSDKAHB_GPIOState *s = CMSDKAHB_GPIO(opaque);

    if (!extract32(s->outenbits, line, 1)) {
	    CMSDKAHB_GPIO_set_int_line(s, line, level);

	    s->data = deposit32(s->data, line, 1, level);

	    CMSDKAHB_GPIO_update_int(s);
    }
}

static void CMSDKAHB_GPIO_set_all_int_lines(CMSDKAHB_GPIOState *s)
{
    int i;

    uint32_t val = (s->data & ~s->outenbits) | (s->dataout & s->outenbits);

    for (i = 0; i < CMSDKAHB_GPIO_PIN_COUNT; i++) {
    	CMSDKAHB_GPIO_set_int_line(s, i, extract32(val, i, 1));
    }

    CMSDKAHB_GPIO_update_int(s);
}

static inline void CMSDKAHB_GPIO_set_all_output_lines(CMSDKAHB_GPIOState *s)
{
    int i;

    for (i = 0; i < CMSDKAHB_GPIO_PIN_COUNT; i++) {
        if (extract32(s->outenbits, i, 1) && s->output[i]) {
    		CMSDKAHB_GPIO_set_int_line(s, i, extract32(s->dataout, i, 1));
            qemu_set_irq(s->output[i], extract32(s->dataout, i, 1));
        }
    }
    CMSDKAHB_GPIO_update_int(s);
}

static uint64_t CMSDKAHB_GPIO_read(void *opaque, hwaddr addr, unsigned int size)
{
    CMSDKAHB_GPIOState *s = opaque;
    uint64_t val = 0;

    switch (addr) {

    case REG_DATA:
    	val = (s->data & ~s->outenbits) | (s->dataout & s->outenbits);
    	break;
    case REG_DATAOUT:
    	val = s->dataout;
    	break;
    case REG_OUTENSET:
    case REG_OUTENCLR:
    	val = s->outenbits;
    	break;
    case REG_ALTFUNCSET:
    case REG_ALTFUNCCLR:
    	val = s->altfunc;
    	break;
    case REG_INTENSET:
    case REG_INTENCLR:
    	val = s->inten;
    	break;
    case REG_INTTYPESET:
    case REG_INTTYPECLR:
    	val = s->inttype;
    	break;
    case REG_INTPOLSET:
    case REG_INTPOLCLR:
    	val = s->intpol;
    	break;
    case REG_INTSTATUS:
    	val = s->intstatus;
    	break;
    case MASKLOWBYTE_START...MASKLOWBYTE_END: //unrealized
    	val = s->masklowbyte;
    	break;
    case MASKHIGHBYTE_START...MASKHIGHBYTE_END: //unrealized
    	val = s->maskhighbyte;
    	break;

    default:
        break;
    }

    return val;
}

static void CMSDKAHB_GPIO_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned int size)
{
    CMSDKAHB_GPIOState *s = CMSDKAHB_GPIO(opaque);

    switch (addr) {
    case REG_DATA:
    case REG_DATAOUT:
    	s->dataout = val;
		CMSDKAHB_GPIO_set_all_output_lines(s);
    	break;
    case REG_OUTENSET:
    	s->outenbits = val;
		CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_OUTENCLR:
    	s->outenbits &= ~val;
		CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_ALTFUNCSET:
    	s->altfunc = val;
    	break;
    case REG_ALTFUNCCLR:
    	s->altfunc &= ~val;
    	break;
    case REG_INTENSET:
    	s->inten = val;
    	CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_INTENCLR:
    	s->inten &= ~val;
    	CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_INTTYPESET:
    	s->inttype = val;
    	CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_INTTYPECLR:
    	s->inttype &= ~val;
    	CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_INTPOLSET:
    	s->intpol = val;
    	CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_INTPOLCLR:
    	s->intpol &= ~val;
    	CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case REG_INTSTATUS:
    	s->intstatus &= ~(val & s->inttype) & s->inten;
    	CMSDKAHB_GPIO_set_all_int_lines(s);
    	break;
    case MASKLOWBYTE_START...MASKLOWBYTE_END: //unrealized
    	s->masklowbyte = val;
    	break;
    case MASKHIGHBYTE_START...MASKHIGHBYTE_END: //unrealized
    	s->maskhighbyte = val;
    	break;

    default:
        break;
    }
}

static const MemoryRegionOps CMSDKAHB_GPIO_ops = {
    .read = CMSDKAHB_GPIO_read,
    .write = CMSDKAHB_GPIO_write,
};

static Property CMSDKAHB_GPIO_properties[] = {
    DEFINE_PROP_UINT32("AltFuncVal", CMSDKAHB_GPIOState, altfunc, 0xFFFF),
    DEFINE_PROP_END_OF_LIST(),
};

static void CMSDKAHB_GPIO_reset(DeviceState *dev)
{
    CMSDKAHB_GPIOState *s = CMSDKAHB_GPIO(dev);

	s->dataout = 0;
	s->outenbits = 0;
	s->inten = 0;
	s->inttype = 0;
	s->intpol = 0;
	s->intstatus = 0;

	CMSDKAHB_GPIO_set_all_int_lines(s);
}

static void CMSDKAHB_GPIO_realize(DeviceState *dev, Error **errp)
{
    CMSDKAHB_GPIOState *s = CMSDKAHB_GPIO(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &CMSDKAHB_GPIO_ops, s, TYPE_CMSDKAHB_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(DEVICE(s), CMSDKAHB_GPIO_set, CMSDKAHB_GPIO_PIN_COUNT);
    qdev_init_gpio_out(DEVICE(s), s->output, CMSDKAHB_GPIO_PIN_COUNT);
    
    sysbus_init_irq(sbd, &s->irq);
}

static void CMSDKAHB_GPIO_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = CMSDKAHB_GPIO_realize;
    dc->reset = CMSDKAHB_GPIO_reset;
    dc->desc = "cmsdk-ahb-gpio";
    device_class_set_props(dc, CMSDKAHB_GPIO_properties);
}

static const TypeInfo CMSDKAHB_GPIO_info = {
    .name          = TYPE_CMSDKAHB_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CMSDKAHB_GPIOState),
    .class_init    = CMSDKAHB_GPIO_class_init,
};

static void CMSDKAHB_GPIO_register_types(void)
{
    type_register_static(&CMSDKAHB_GPIO_info);
}

type_init(CMSDKAHB_GPIO_register_types)
