#include "ch.h"
#include "hal.h"

void demo_led_init(void)
{
    /*
     * PWM configuration structure.
     * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
     * the active state is a logic one.
     */
    static const PWMConfig pwmcfg = {
        100000,                                   /* 100kHz PWM clock frequency.  */
        128,                                      /* PWM period is 128 cycles.    */
        NULL,
        {
            {PWM_OUTPUT_ACTIVE_HIGH, NULL},
            {PWM_OUTPUT_ACTIVE_HIGH, NULL},
            {PWM_OUTPUT_ACTIVE_HIGH, NULL},
            {PWM_OUTPUT_ACTIVE_HIGH, NULL}
        },
        /* HW dependent part.*/
        0,
        0
    };

    /*
     * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
     */
    pwmStart(&PWMD4, &pwmcfg);
    palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));      /* Green.   */
    palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));      /* Orange.  */
    palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_ALTERNATE(2));      /* Red.     */
    palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));      /* Blue.    */
}

void demo_led_set(int led, int brightness)
{
    switch(led) {
        case 3:
            pwmEnableChannel(&PWMD4, 1, (pwmcnt_t) brightness);
            break;
        case 4:
            pwmEnableChannel(&PWMD4, 0, (pwmcnt_t) brightness);
            break;
        case 5:
            pwmEnableChannel(&PWMD4, 2, (pwmcnt_t) brightness);
            break;
        case 6:
            pwmEnableChannel(&PWMD4, 3, (pwmcnt_t) brightness);
            break;
    }
}
