#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/boards.h"
#include "cpu.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "hw/qdev-properties.h"
#include "hw/char/pl011.h"
#include "hw/net/greth.h"
#include "hw/sd/sd.h"
#include "hw/sd/keyasic_sd.h"
#include "hw/irq.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "hw/arm/satellite_soc.h"
#include "hw/qdev-clock.h"
#include "hw/arm/boot.h"

#define SYSCLK_FRQ (80 * 1000 * 1000)
#define NUM_IRQ_LINES 64

typedef struct {
    MachineState parent;

    ARMv7MState *cpu;

    NVICState nvic;

} B5023VS016MachineState;

#define TYPE_B5023VS016_MACHINE MACHINE_TYPE_NAME("B5023VS016")
#define B5023VS016_MACHINE(obj) \
    OBJECT_CHECK(B5023VS016MachineState, obj, TYPE_B5023VS016_MACHINE)

static void B5023VS016_init(MachineState *machine)
{
    DeviceState *dev;
    Clock *sysclk;

    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    dev = qdev_new(TYPE_SATELLITE_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m0"));
    qdev_connect_clock_in(dev, "sysclk", sysclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       0, FLASH_SIZE);
}

static void B5023VS016_reset(MachineState *machine, ShutdownCause reason)
{
    qemu_devices_reset(reason);
}

static void B5023VS016_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Integrated circuit 5023BC016";
    mc->alias = "SATELLITE";

    mc->init = B5023VS016_init;
    mc->reset = B5023VS016_reset;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m0");
}

static const TypeInfo B5023VS016_info = {
    .name = TYPE_B5023VS016_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(B5023VS016MachineState),
    .class_init = B5023VS016_class_init,
};

static void B5023VS016_machines_init(void)
{
    type_register_static(&B5023VS016_info);
}

type_init(B5023VS016_machines_init)
