#include "ch.h"
#include "hal.h"
#include "test.h"
#include "usbcfg.h"


/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palSetPad(GPIOD, GPIOD_LED3);   /* Orange.  */
        chThdSleepMilliseconds(500);
        palClearPad(GPIOD, GPIOD_LED3); /* Orange.  */
        chThdSleepMilliseconds(500);
    }
}

/*
 * Application entry point.
 */
int main(void)
{
    halInit();
    chSysInit();

    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    /*
     * Creates the example thread.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    /*
     * Normal main() thread activity, in this demo it does nothing except
     * sleeping in a loop and check the button state.
     */
    while (true) {
        if (palReadPad(GPIOA, GPIOA_BUTTON)) {
            TestThread(&SDU1);
        }
        chThdSleepMilliseconds(500);
    }
}
