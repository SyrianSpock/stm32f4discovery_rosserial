#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include "ros.h"
#include "std_msgs/String.h"


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

    /* ROS setup */
    ros::NodeHandle ros_node;
    ros_node.initNode();

    /* ROS publisher */
    std_msgs::String str_msg;
    ros::Publisher chatter("chatter", &str_msg);
    ros_node.advertise(chatter);

    char hello[13] = "hello world!";

    while (true) {
        str_msg.data = hello;
        chatter.publish(&str_msg);

        ros_node.spinOnce();
        chThdSleepMilliseconds(100);
        palTogglePad(GPIOD, GPIOD_LED4); // Green
    }
}
