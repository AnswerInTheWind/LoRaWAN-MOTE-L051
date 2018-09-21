/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#if defined( SX1262DVK1DAS )
#define BOARD_TCXO_WAKEUP_TIME                      5
#else
#define BOARD_TCXO_WAKEUP_TIME                      0
#endif

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PB_0

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5

#if defined( SX1261DVK1BAS ) || defined( SX1262DVK1CAS ) || defined( SX1262DVK1DAS )

#define RADIO_NSS                                   PA_4
#define RADIO_BUSY                                  PA_3
#define RADIO_DIO_1                                 PB_1

#elif defined( SX1272MB2DAS) || defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )

#define RADIO_NSS                                   PA_4

#define RADIO_DIO_0                                 PB_0
#define RADIO_DIO_1                                 PB_1
//#define RADIO_DIO_2                                 PB_5
#define RADIO_DIO_3                                 PB_2
//#define RADIO_DIO_4                                 PA_9
//#define RADIO_DIO_5                                 PC_7

#define RADIO_ANT_SWITCH                            NC

#endif

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define SWCLK                                       PA_14
#define SWDAT                                       PA_13

#define I2C_SCL                                     NC
#define I2C_SDA                                     NC

#define UART_TX                                     PB_6
#define UART_RX                                     PB_7

#define LED_GREEN                                   PC_13
#define LED_RED                                     PA_0

#define KEY                                         PA_8

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            NC
#define RADIO_DBG_PIN_RX                            NC

#endif // __BOARD_CONFIG_H__
