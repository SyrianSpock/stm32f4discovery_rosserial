#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void(*button_callback)(void);

/** Starts the thread that will read the button. */
void demo_button_start(button_callback cb);

#ifdef __cplusplus
}
#endif

#endif
