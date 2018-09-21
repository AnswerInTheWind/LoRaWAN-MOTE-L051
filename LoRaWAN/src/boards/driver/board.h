/*!
 * \file      board.h
 *
 * \brief     Target board general functions implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#define USE_KEY
#define USE_OLED
#define xUSE_LOW_POWER
#define USE_MSG_DBG

#include <stdint.h>
#include <stdio.h>

#include "uart-board.h"
#include "stm32l0xx_hal.h"

#include "stm32l0xx.h"
#include "utilities.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "board-config.h"
#include "rtc-board.h"
#include "key-board.h"

#if defined( SX1261DVK1BAS ) || defined( SX1262DVK1CAS ) || defined( SX1262DVK1DAS )
    #include "sx126x-board.h"
#elif defined( SX1272MB2DAS)
    #include "sx1272-board.h"
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    #include "sx1276-board.h"
#endif

#if defined( USE_KEY )
    #include "key-board.h"
#endif

#if defined( USE_OLED )
    #include "oled-board.h"
#endif


/*!
 * Possible power sources
 */
enum BoardPowerSources
{
    USB_POWER = 0,
    BATTERY_POWER,
};

/*!
 * Board Version
 */
typedef union BoardVersion_u
{
    struct BoardVersion_s
    {
        uint8_t Rfu;
        uint8_t Revision;
        uint8_t Minor;
        uint8_t Major;
    }Fields;
    uint32_t Value;
}BoardVersion_t;


#ifdef USE_MSG_DBG
    #define LOG_PRINTF(args...) printf(args)
    #define MSG_DBG(format, ...) LOG_PRINTF("[ %d ] "format"",HAL_GetTick(),##__VA_ARGS__)
#else
    extern int invalid_MSG_DBG(const char *format,...);
    #define MSG_DBG invalid_MSG_DBG
#endif



/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the mcu.
 */
void BoardInitMcu( void );

/*!
 * \brief Resets the mcu.
 */
void BoardResetMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Gets the current potentiometer level value
 *
 * \retval value  Potentiometer level ( value in percent )
 */
uint8_t BoardGetPotiLevel( void );

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value  battery voltage in volts
 */
uint32_t BoardGetBatteryVoltage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source [0: USB_POWER, 1: BATTERY_POWER]
 */
uint8_t GetBoardPowerSource( void );

/*!
 * \brief Get the board version
 *
 * \retval value  Version
 */
BoardVersion_t BoardGetVersion( void );

#endif // __BOARD_H__
