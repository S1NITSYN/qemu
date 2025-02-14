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
#define MASKLOWBYTE		0x400// ... 0x7FC)
#define MASKHIGHBYTE	0x800// ... 0xBFC)

static void CMSDKAHB_GPIO_set(void *opaque, int line, int level)
{
    
    CMSDKAHB_GPIOState *s = CMSDKAHB_GPIO(opaque);

    s->data = deposit32(s->data, line, 1, level);
}

static inline void CMSDKAHB_GPIO_set_all_output_lines(CMSDKAHB_GPIOState *s)
{
    int i;

    for (i = 0; i < CMSDKAHB_GPIO_PIN_COUNT; i++) {
        if (extract32(s->outenbits, i, 1) && s->output[i]) {
            qemu_set_irq(s->output[i], extract32(s->dataout, i, 1));
        }
    }
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
    	val = s->outenbits;
    	break;

    case REG_OUTENCLR:
    	val = s->outenbits;
    	break;
    case REG_ALTFUNCSET:
    	val = s->altfuncset;
    	break;
    case REG_ALTFUNCCLR:
    	val = s->altfuncclr;
    	break;
    case REG_INTENSET:
    	val = s->intenset;
    	break;
    case REG_INTENCLR:
    	val = s->intenclr;
    	break;
    case REG_INTTYPESET:
    	val = s->inttypeset;
    	break;
    case REG_INTTYPECLR:
    	val = s->inttypeclr;
    	break;
    case REG_INTPOLSET:
    	val = s->intpolset;
    	break;
    case REG_INTPOLCLR:
    	val = s->intpolclr;
    	break;
    case REG_INTSTATUS:
    	val = s->intstatus;
    	break;
    case MASKLOWBYTE:
    	/*
			Для этих регистров, если они будут использщоваться,
			сделать срез конкретно на этот адрес, а значение
			хранить в отдельной переменной
    	*/
    	val = s->masklowbyte;
    	break;
    case MASKHIGHBYTE:
    	/*
			Для этих регистров, если они будут использщоваться,
			сделать срез конкретно на этот адрес, а значение
			хранить в отдельной переменной
    	*/
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
    	s->dataout = val; //после обновления dataout, нужно обновлять qemu_irq output поднимать или опускать эти прерывания
		CMSDKAHB_GPIO_set_all_output_lines(s);
    	break;
    case REG_OUTENSET:
    	s->outenbits = val;
		CMSDKAHB_GPIO_set_all_output_lines(s);
    	break;
    case REG_OUTENCLR:
    	s->outenbits &= ~val;
		CMSDKAHB_GPIO_set_all_output_lines(s);
    	break;
    case REG_ALTFUNCSET:	//Используется для включения альтернативного функционала(i2c, spi)???
    	s->altfuncset = val;
    	break;
    case REG_ALTFUNCCLR:
    	s->altfuncclr &= ~val;
    	break;
    case REG_INTENSET:
    	s->intenset = val;
    	break;
    case REG_INTENCLR:
    	s->intenclr &= ~val;
    	break;
    case REG_INTTYPESET:
    	s->inttypeset = val;
    	break;
    case REG_INTTYPECLR:
    	s->inttypeclr &= ~val;
    	break;
    case REG_INTPOLSET:
    	s->intpolset = val;
    	break;
    case REG_INTPOLCLR:
    	s->intpolclr &= ~val;
    	break;
    case REG_INTSTATUS:
    	s->intstatus = val;
    	break;
    case MASKLOWBYTE:
    	s->masklowbyte = val;
    	break;
    case MASKHIGHBYTE:
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

static void CMSDKAHB_GPIO_reset(DeviceState *dev)
{
    CMSDKAHB_GPIOState *s = CMSDKAHB_GPIO(dev);

	s->dataout = 0;
	s->outenbits = 0;
	s->altfuncset = 0; //propertie value
	s->altfuncclr = 0; //propertie value
	s->intenset = 0;
	s->inttypeset = 0;
	s->inttypeclr = 0;
	s->intpolset = 0;
	s->intpolclr = 0;
	s->intstatus = 0;
	//s->masklowbyte = 0;
	//s->maskhighbyte = 0;

	CMSDKAHB_GPIO_set_all_output_lines(s);
}

static void CMSDKAHB_GPIO_realize(DeviceState *dev, Error **errp)
{
    CMSDKAHB_GPIOState *s = CMSDKAHB_GPIO(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &CMSDKAHB_GPIO_ops, s, TYPE_CMSDKAHB_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(DEVICE(s), CMSDKAHB_GPIO_set, CMSDKAHB_GPIO_PIN_COUNT);
    qdev_init_gpio_out(DEVICE(s), s->output, CMSDKAHB_GPIO_PIN_COUNT);
}

static void CMSDKAHB_GPIO_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = CMSDKAHB_GPIO_realize;
    dc->reset = CMSDKAHB_GPIO_reset;
    dc->desc = "ZVENOM";
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
