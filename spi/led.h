#ifndef __LED_H
#define __LED_H

#include "hal_gpio_driver.h"

//#define EXTIx_IRQn                  EXTI0_IRQn
//#define EXTIx_IRQHandler            EXTI0_IRQHandler

#define GPIO_BUTTON_PIN             0
//#define GPIO_BUTTON_PORT            GPIOA

/* LEDs GPIO definitions */

#define GPIO_PIN_12                 12
#define GPIO_PIN_13                 13
#define GPIO_PIN_14                 14
#define GPIO_PIN_15                 15

#define LED_GREEN                   GPIO_PIN_12
#define LED_ORANGE                  GPIO_PIN_13
#define LED_RED                     GPIO_PIN_14
#define LED_BLUE                    GPIO_PIN_15

/**
* @brief  Initializes the LEDs
* @param None
* @retval None
*/
void led_init(void);

/**
* @brief  Turns ON hte led which is connected on the given pin
* @param  *GPIOx : Base address of the GPIO Port
* @param  pin    : pin number of the LED
* @retval None
*/
void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Turns OFF the led which is connected on the given pin
* @param  *GPIOx : Base address of the GPIO Port
* @param  pin    : pin number of the LED
* @retval None
*/
void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Toggles the led which is connected on the given pin
* @param  *GPIOx : Base address of the GPIO Port
* @param  pin    : pin number of the LED
* @retval None
*/
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif
