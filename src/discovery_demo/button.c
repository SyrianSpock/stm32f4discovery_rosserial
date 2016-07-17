#include <ch.h>
#include <hal.h>
#include "button.h"

static void wait_for_state(bool state)
{
    while (palReadPad(GPIOA, GPIOA_BUTTON) != state) {
        chThdSleepMilliseconds(10);
    }
}

static THD_FUNCTION(button_thd, p)
{
    void (*callback)(void) = p;

    chRegSetThreadName("button");

    while (true) {
        /* Wait for a button press. */
        wait_for_state(true);

        /* Debounce button. */
        chThdSleepMilliseconds(10);

        /* Wait for button release. */
        wait_for_state(false);

        /* Debounce button. */
        chThdSleepMilliseconds(10);

        /* Call back into user provided code. */
        callback();
    }
}

void demo_button_start(button_callback cb)
{
    static THD_WORKING_AREA(wa, 1024);
    chThdCreateStatic(wa, sizeof(wa), NORMALPRIO, button_thd, cb);
}
