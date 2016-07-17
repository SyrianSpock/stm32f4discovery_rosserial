#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include "discovery_demo/button.h"

#include "ros.h"
#include "std_msgs/String.h"


ros::NodeHandle ros_node;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

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

void button_cb(void)
{
    char hello[16] = "Button pressed!";
    str_msg.data = hello;
    chatter.publish(&str_msg);
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

    /* Create blinker thread */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);


    /* ROS setup */
    ros_node.initNode();

    /* ROS publisher */
    ros_node.advertise(chatter);

    /* Setup Discovery board demo */
    demo_button_start(button_cb);

    while (true) {
        ros_node.spinOnce();
        chThdSleepMilliseconds(10);
        palTogglePad(GPIOD, GPIOD_LED4); // Green
    }
}
