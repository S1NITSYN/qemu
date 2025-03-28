#ifndef HW_NET_CAN_CAN_SATELLITE
#define HW_NET_CAN_CAN_SATELLITE

#include "hw/sysbus.h"
#include "hw/irq.h"
#include "net/can_emu.h"
#include "can_sja1000.h"

#define can_satellite_SJA_RANGE      0x10000
#define can_satellite_BYTES_PER_SJA  0x20

struct CanSatelliteState {
    /*< private >*/
    SysBusDevice parent;
    /*< public >*/
    MemoryRegion    sja_io;

    CanSJA1000State sja_state;
    qemu_irq        irq;
    uint8_t			irq_level;

    CanBusState     *canbus;
};


typedef struct CanSatelliteState CanSatelliteState;

#define TYPE_CAN_DEV "can_satellite"
DECLARE_INSTANCE_CHECKER(CanSatelliteState, CAN_SATELLITE_DEV,
                         TYPE_CAN_DEV)

#endif /*HW_NET_CAN_CAN_SATELLITE*/
