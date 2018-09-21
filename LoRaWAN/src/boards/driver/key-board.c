/*
(C)2015 NPLink

Description: Key driver implementation, used for key press.

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Derek Chen
*/
#include <stdio.h>
#include "key-board.h"

extern key_t key;

//* Private function prototypes -----------------------------------------------*/
void key_interrupt_callback(void);

/* Private functions ---------------------------------------------------------*/
void key_init( void )
{
    //初始化key为输入/上拉
    GpioInit(&key.key_pin,KEY_PINNAME,PIN_INPUT,PIN_PUSH_PULL,PIN_PULL_UP,1);
    
    //设置key为中断/下降沿触发/次高优先级,回调函数为key_interrupt_callback
    GpioSetInterrupt( &key.key_pin, IRQ_FALLING_EDGE, IRQ_HIGH_PRIORITY, key_interrupt_callback );

    key.key_Pressed_callback = NULL;
}

void key_DeInit( void )
{
   GpioInit( &key.key_pin, KEY_PINNAME, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
}

//按键的回调函数
void key_interrupt_callback(void)
{
    if(key.key_Pressed_callback != NULL)
        key.key_Pressed_callback();
}
