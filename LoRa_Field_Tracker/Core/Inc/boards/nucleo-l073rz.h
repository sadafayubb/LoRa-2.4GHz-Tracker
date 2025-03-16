/*
 * nucleo-l073rz.h
 *
 *  Created on: 14 Mar 2025
 *      Author: sadaf
 */

#ifndef __BOARD__NUCLEO__L073RZ_H__
#define __BOARD__NUCLEO__L073RZ_H__

#ifdef SHIELD_PCB_E394V02A  // Replace with your shield’s define (check schematic)

// SPI Pins (SX1280 SPI1 on Nucleo-L073RZ)
#define RADIO_NSS_PIN       GPIO_PIN_4   // PA4 (GPIO OUTPUT)(user-defined)
#define RADIO_NSS_PORT      GPIOA

#define RADIO_MOSI_PIN      GPIO_PIN_7   // PA7 (MOSI)
#define RADIO_MOSI_PORT     GPIOA

#define RADIO_MISO_PIN      GPIO_PIN_6   // PA6 (MISO)
#define RADIO_MISO_PORT     GPIOA

#define RADIO_SCK_PIN       GPIO_PIN_3   // PB3 (SCK)
#define RADIO_SCK_PORT      GPIOB

// Radio Control Pins (do need these control pins to configure)
#define RADIO_nRESET_PIN    GPIO_PIN_0   // PA0 (NRESET) (user-defined)
#define RADIO_nRESET_PORT   GPIOA

#define RADIO_BUSY_PIN      GPIO_PIN_2   // PB2	(BUSY) (user-defined)
#define RADIO_BUSY_PORT     GPIOX

#define RADIO_DIO1_PIN      GPIO_PIN_4   // PB4 (DIO1,DIO2,DIO3) (user-defined)
#define RADIO_DIO1_PORT     GPIOB

#define RADIO_DIOx_PIN      GPIO_PIN_4
#define RADIO_DIOx_PORT     GPIOB

// UART Pins (for debug output - ST-Link Virtual COM Port)
#define USART_TX_PIN        GPIO_PIN_2   // PA2 (USART2_TX)
#define USART_TX_PORT       GPIOA

#define USART_RX_PIN        GPIO_PIN_3   // PA3 (USART2_RX)
#define USART_RX_PORT       GPIOA

// LEDs (Nucleo-L073RZ onboard LEDs ??)

#define ANT_SW_PIN          GPIO_PIN_0 // (user-defined)
#define ANT_SW_PORT         GPIOB

#define LED_RX_PIN          GPIO_PIN_0 // (user-defined)
#define LED_RX_PORT         GPIOC

#define LED_TX_PIN          GPIO_PIN_1 // (user-defined)
#define LED_TX_PORT         GPIOC


#endif // __BOARD__NUCLEO__L073RZ_H__
