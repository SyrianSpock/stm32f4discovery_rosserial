#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include "discovery_demo/accelerometer.h"
#include "discovery_demo/button.h"

#include "ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"


ros::NodeHandle ros_node;

std_msgs::String str_msg;
geometry_msgs::Vector3 acc_msg;
ros::Publisher button_pub("button", &str_msg);
ros::Publisher acc_pub("accelerometer", &acc_msg);

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waBlinkThd, 128);
static THD_FUNCTION(BlinkThd, arg)
{
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
    button_pub.publish(&str_msg);
}

void accelerometer_cb(void)
{
    float acc[3];
    demo_acc_get_acc(acc);

    acc_msg.x = acc[0];
    acc_msg.y = acc[1];
    acc_msg.z = acc[2];

    acc_pub.publish(&acc_msg);
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
    chThdCreateStatic(waBlinkThd, sizeof(waBlinkThd), NORMALPRIO, BlinkThd, NULL);

    /* ROS setup */
    ros_node.initNode();

    /* ROS publishers */
    ros_node.advertise(button_pub);
    ros_node.advertise(acc_pub);

    /* Setup Discovery board demo */
    demo_button_start(button_cb);
    demo_acc_start(accelerometer_cb);

    while (true) {
        ros_node.spinOnce();
        chThdSleepMilliseconds(100);
        palTogglePad(GPIOD, GPIOD_LED4); // Green
    }
}
