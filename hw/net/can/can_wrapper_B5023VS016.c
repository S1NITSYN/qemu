#include "qemu/osdep.h"
#include "qemu/event_notifier.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qemu/thread.h"
#include "qemu/sockets.h"
#include "qapi/error.h"
#include "chardev/char.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "net/can_emu.h"

#include "can_sja1000.h"
#include "qom/object.h"
#include "can_wrapper_B5023VS016.h"

static void can_satellite_irq_handler(void *opaque, int irq_num, int level)
{
    CanSatelliteState *d = opaque;

    if (d->irq_level == level) {
        return;
    }

    d->irq_level = level;
    qemu_set_irq(d->irq, d->irq_level);
}

static void can_satellite_reset(DeviceState *dev)
{
    CanSatelliteState *d = CAN_SATELLITE_DEV(dev);
    CanSJA1000State *s = &d->sja_state;

    can_sja_hardware_reset(s);
}

static uint64_t can_satellite_sja_io_read(void *opaque, hwaddr addr, unsigned size)
{
    CanSatelliteState *d = opaque;
    CanSJA1000State *s = &d->sja_state;
    hwaddr actual_address = (addr / 4);

    if (actual_address >= can_satellite_BYTES_PER_SJA) {
        return 0;
    }

    return can_sja_mem_read(s, actual_address, size);
}

static void can_satellite_sja_io_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size)
{
    CanSatelliteState *d = opaque;
    CanSJA1000State *s = &d->sja_state;
    hwaddr actual_address = (addr / 4);

    if (actual_address >= can_satellite_BYTES_PER_SJA) {
        return;
    }

    can_sja_mem_write(s, actual_address, data, size);
}

static const MemoryRegionOps can_satellite_sja_io_ops = {
    .read = can_satellite_sja_io_read,
    .write = can_satellite_sja_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /*.impl = {
        .max_access_size = 1,
    },*/
};

static void can_satellite_realize(DeviceState *dev, Error **errp)
{
    CanSatelliteState *d = CAN_SATELLITE_DEV(dev);
    CanSJA1000State *s = &d->sja_state;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    d->irq = qemu_allocate_irq(can_satellite_irq_handler, d, 0);

    can_sja_init(s, d->irq);

    if (can_sja_connect_to_bus(s, d->canbus) < 0) {
        error_setg(errp, "can_sja_connect_to_bus failed");
        return;
    }

    memory_region_init_io(&d->sja_io, OBJECT(dev), &can_satellite_sja_io_ops,
                          d, "can_satellite-sja", can_satellite_SJA_RANGE);
    sysbus_init_mmio(sbd, &d->sja_io);
    sysbus_init_irq(sbd, &d->irq);
}

/*static void can_satellite_exit(PCIDevice *pci_dev)
{
    CanSatelliteState *d = CAN_SATELLITE_DEV(pci_dev);
    CanSJA1000State *s = &d->sja_state;

    can_sja_disconnect(s);

    qemu_free_irq(d->irq);
}*/

static void can_satellite_instance_init(Object *obj)
{
    CanSatelliteState *d = CAN_SATELLITE_DEV(obj);

    object_property_add_link(obj, "canbus", TYPE_CAN_BUS,
                             (Object **)&d->canbus,
                             qdev_prop_allow_set_link_before_realize,
                             0);
}

static void can_satellite_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = can_satellite_realize;
    //dc->exit = can_satellite_exit;
    dc->desc = "Satellite CAN device";
    dc->reset = can_satellite_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo can_satellite_info = {
    .name          = TYPE_CAN_DEV,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CanSatelliteState),
    .class_init    = can_satellite_class_init,
    .instance_init = can_satellite_instance_init,
};

static void can_satellite_register_types(void)
{
    type_register_static(&can_satellite_info);
}

type_init(can_satellite_register_types)
