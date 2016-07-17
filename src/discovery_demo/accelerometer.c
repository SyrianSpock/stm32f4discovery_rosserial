#include "ch.h"
#include "hal.h"

#include "lis302dl.h"

#include "discovery_demo/accelerometer.h"

accelerometer_sample_t acc_sample;
accelerometer_callback acc_callback;


static THD_WORKING_AREA(waAcceleroThd, 128);
static THD_FUNCTION(AcceleroThd, arg)
{
    systime_t time;

    (void)arg;
    chRegSetThreadName("accelerometer");

    /* Reader thread loop.*/
    while (TRUE) {
        unsigned i;

        time = chVTGetSystemTime();

        /* Reading MEMS accelerometer X, Y and Z registers.*/
        acc_sample.acceleration[0] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTX);
        acc_sample.acceleration[1] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTY);
        acc_sample.acceleration[2] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTZ);

        acc_callback();

        /* Waiting until the next 100 milliseconds time interval.*/
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

void demo_acc_start(accelerometer_callback callback)
{
    static const SPIConfig spi1cfg = {
        NULL,
        /* HW dependent part.*/
        GPIOE,
        GPIOE_CS_SPI,
        SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
    };
    spiStart(&SPID1, &spi1cfg);
    acc_callback = callback;

    chThdSleepMilliseconds(500);

    /* LIS302DL initialization.*/
    lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG1, 0x43);
    lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG2, 0x00);
    lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG3, 0x00);

    chThdCreateStatic(waAcceleroThd,
                      sizeof(waAcceleroThd),
                      NORMALPRIO + 10,
                      AcceleroThd,
                      NULL);
}

void demo_acc_get_acc(float *acc)
{
    chSysLock();
    acc[0] = acc_sample.acceleration[0];
    acc[1] = acc_sample.acceleration[1];
    acc[2] = acc_sample.acceleration[2];
    chSysUnlock();
}
