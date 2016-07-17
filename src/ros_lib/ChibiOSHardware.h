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
        ChibiOSHardware(BaseSequentialStream* io)
        {
            iostream = io;
        }
        ChibiOSHardware()
        {
            iostream = (BaseSequentialStream *)&SDU1;
        }

        void init()
        {
        }

        int read()
        {
            return chSequentialStreamGet(iostream);
        };

        void write(uint8_t* data, int length)
        {
            chSequentialStreamWrite(iostream, data, length);
        }

        unsigned long time()
        {
            return MS2ST(chVTGetSystemTimeX());;
        }

    protected:
        BaseSequentialStream* iostream;
};

#ifdef __cplusplus
}
#endif

#endif /* ROS_CHIBIOS_HARDWARE_H */
