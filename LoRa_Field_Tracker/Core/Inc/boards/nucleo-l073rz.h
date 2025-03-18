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
#define RADIO_NSS_PIN       GPIO_PIN_8   // PA8 , NSS (Chip Select)
#define RADIO_NSS_PORT      GPIOA

#define RADIO_MOSI_PIN      GPIO_PIN_7   // PA7 (MOSI)
#define RADIO_MOSI_PORT     GPIOA

#define RADIO_MISO_PIN      GPIO_PIN_6   // PA6 (MISO)
#define RADIO_MISO_PORT     GPIOA

#define RADIO_SCK_PIN       GPIO_PIN_5   // PA5 (SCK)
#define RADIO_SCK_PORT      GPIOA

// Radio Control Pins (do need these control pins so configure them)
#define RADIO_nRESET_PIN    GPIO_PIN_0   // PA0 (Hardware reset for SX1280)
#define RADIO_nRESET_PORT   GPIOA

#define RADIO_BUSY_PIN      GPIO_PIN_3   // PB3	(BUSY) BUSY Signal
#define RADIO_BUSY_PORT     GPIOB

#define RADIO_DIO1_PIN      GPIO_PIN_4   // PB4 (triggers when a ranging/radio event occurs (e.g., packet received).)
#define RADIO_DIO1_PORT     GPIOB

#define RADIO_DIOx_PIN      GPIO_PIN_4   // additional DIO pins for advanced use
#define RADIO_DIOx_PORT     GPIOB


// Extra Stuff We don't really need

#define USART_TX_PIN        GPIO_PIN_2 // (A2)
#define USART_TX_PORT       GPIOA

#define USART_RX_PIN        GPIO_PIN_3 // (A3)
#define USART_RX_PORT       GPIOA

#define ANT_SW_PIN          GPIO_PIN_0 // (B0) Controls external RF switch
#define ANT_SW_PORT         GPIOB

#define LED_RX_PIN          GPIO_PIN_0 // (C0)
#define LED_RX_PORT         GPIOC

#define LED_TX_PIN          GPIO_PIN_1 // (C1)
#define LED_TX_PORT         GPIOC

#endif

#endif // __BOARD__NUCLEO__L073RZ_H__
