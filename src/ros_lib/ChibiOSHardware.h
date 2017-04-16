#ifndef ROS_CHIBIOS_HARDWARE_H
#define ROS_CHIBIOS_HARDWARE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch.h"
#include "hal.h"
#include "chstreams.h"


class ChibiOSHardware {
    public:
        ChibiOSHardware(BaseChannel* io)
        {
            iostream = io;
        }
        ChibiOSHardware()
        {
            iostream = (BaseChannel *)&SDU1;
        }

        void init()
        {
        }

        int read()
        {
            return chnGetTimeout(iostream, TIME_IMMEDIATE);
        };

        void write(uint8_t* data, int length)
        {
            chnWrite(iostream, data, length);
        }

        unsigned long time()
        {
            return ST2MS(chVTGetSystemTimeX());;
        }

    protected:
        BaseChannel* iostream;
};

#ifdef __cplusplus
}
#endif

#endif /* ROS_CHIBIOS_HARDWARE_H */
