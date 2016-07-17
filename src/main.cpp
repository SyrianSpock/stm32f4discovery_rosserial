#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include "discovery_demo/accelerometer.h"
#include "discovery_demo/button.h"
#include "discovery_demo/leds.h"

#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Vector3.h"


void button_cb(void);
void accelerometer_cb(void);
void led_red_cb(const std_msgs::UInt16& led_green_msg);
void led_blue_cb(const std_msgs::UInt16& led_green_msg);
void led_green_cb(const std_msgs::UInt16& led_green_msg);
void led_orange_cb(const std_msgs::UInt16& led_green_msg);


ros::NodeHandle ros_node;

std_msgs::String str_msg;
geometry_msgs::Vector3 acc_msg;
ros::Publisher button_pub("button", &str_msg);
ros::Publisher acc_pub("accelerometer", &acc_msg);

ros::Subscriber<std_msgs::UInt16> led_red_sub("led/red", &led_red_cb);
ros::Subscriber<std_msgs::UInt16> led_blue_sub("led/blue", &led_blue_cb);
ros::Subscriber<std_msgs::UInt16> led_green_sub("led/green", &led_green_cb);
ros::Subscriber<std_msgs::UInt16> led_orange_sub("led/orange", &led_orange_cb);

/*
 * Callbacks
 */
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

void led_red_cb(const std_msgs::UInt16& led_msg)
{
    demo_led_set(DEMO_LED_RED, led_msg.data);
}

void led_blue_cb(const std_msgs::UInt16& led_msg)
{
    demo_led_set(DEMO_LED_BLUE, led_msg.data);
}

void led_green_cb(const std_msgs::UInt16& led_msg)
{
    demo_led_set(DEMO_LED_GREEN, led_msg.data);
}

void led_orange_cb(const std_msgs::UInt16& led_msg)
{
    demo_led_set(DEMO_LED_ORANGE, led_msg.data);
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

    /* ROS setup */
    ros_node.initNode();

    /* ROS publishers */
    ros_node.advertise(button_pub);
    ros_node.advertise(acc_pub);

    /* ROS subscribers */
    ros_node.subscribe(led_red_sub);
    ros_node.subscribe(led_blue_sub);
    ros_node.subscribe(led_green_sub);
    ros_node.subscribe(led_orange_sub);

    /* Setup Discovery board demo */
    demo_led_init();
    demo_button_start(button_cb);
    demo_acc_start(accelerometer_cb);

    while (true) {
        ros_node.spinOnce();
        chThdSleepMilliseconds(100);
    }
}
