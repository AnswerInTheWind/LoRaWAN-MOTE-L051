#ifndef __KEY_BOARD_H__
#define __KEY_BOARD_H__

#include "gpio.h"

#define KEY_PINNAME                                 PA_8

typedef struct 
{
    Gpio_t key_pin;
    void (*key_Pressed_callback)(void);
}key_t;

/*!
 * \brief key init
 */
extern void key_init(void);

/*!
 * \brief key Deinit
 */
extern void key_DeInit( void );

#endif
