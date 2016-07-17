#ifndef LEDS_H
#define LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

#define DEMO_LED_ORANGE 3
#define DEMO_LED_GREEN  4
#define DEMO_LED_RED    5
#define DEMO_LED_BLUE   6

void demo_led_init(void);
void demo_led_set(int led, int brightness);

#ifdef __cplusplus
}
#endif

#endif /* LEDS_H */
