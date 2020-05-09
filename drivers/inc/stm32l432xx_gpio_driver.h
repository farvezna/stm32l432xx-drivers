/*
 * stm32l432xx_gpio_driver.h
 *
 *  Created on: May 8, 2020
 *      Author: farve
 */

#ifndef INC_STM32L432XX_GPIO_DRIVER_H_
#define INC_STM32L432XX_GPIO_DRIVER_H_

#include "stm32l432xx.h"

/*
 * Pin configuration structure for GPIO pin
 */

typedef struct
{
    uint8_t GPIO_PinNumber;         /*!< possible values from @GPIO_PIN_NUMBERS >*/
    uint8_t GPIO_PinMode;           /*!< possible values from @GPIO_PIN_MODES >*/
    uint8_t GPIO_PinSpeed;          /*!< possible values from @GPIO_PIN_SPEEDS >*/
    uint8_t GPIO_PinPuPdControl;    /*!< possible values from @GPIO_PIN_PUPD_CTRL >*/
    uint8_t GPIO_PinOPType;         /*!< possible values from @GPIO_PIN_OP_TYPE >*/
    uint8_t GPIO_PinAltFunMode;     /*!< possible values from @GPIO_PIN_ATLFUN >*/
} GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
    GPIO_RegDef_t *pGPIOx; //holds base address of the GPIO port to which pin belongs
    GPIO_PinConfig_t GPIO_PinConfig; //holds GPIO pin configuration settings

} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG    3
// mode > 3 --- interrupt
#define GPIO_MODE_IT_FT     4   //interrupt-falling edge trigger
#define GPIO_MODE_IT_RT     5   //interrupt-rising edge trigger
#define GPIO_MODE_IT_RFT    6   //interrupt-rising and falling edge trigger

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin output types
 */
#define GPIO_OP_TYPE_PP     0   //output type-push pull
#define GPIO_OP_TYPE_OD     1   //output type-open drain

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin speeds
 */
#define GPIO_SPEED_LOW          0
#define GPIO_SPEED_MEDIUM       1
#define GPIO_SPEED_HIGH         2
#define GPIO_SPEED_VERYHIGH     3

/*
 * @GPIO_PIN_PUPD_CTRL
 * GPIO pin pull-up and pull-down configurations
 */
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2

/*
 * @GPIO_PIN_ALTFUN
 * GPIO pin alternate function configs
 */
#define GPIO_ALTFUN_AF0     0
#define GPIO_ALTFUN_AF1     1
#define GPIO_ALTFUN_AF2     2
#define GPIO_ALTFUN_AF3     3
#define GPIO_ALTFUN_AF4     4
#define GPIO_ALTFUN_AF5     5
#define GPIO_ALTFUN_AF6     6
#define GPIO_ALTFUN_AF7     7
#define GPIO_ALTFUN_AF8     8
#define GPIO_ALTFUN_AF9     9
#define GPIO_ALTFUN_AF10    10
#define GPIO_ALTFUN_AF11    11
#define GPIO_ALTFUN_AF12    12
#define GPIO_ALTFUN_AF13    13
#define GPIO_ALTFUN_AF14    14
#define GPIO_ALTFUN_AF15    15

/*
 * API prototypes for driver
 */
/****************************************APIs******************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read/write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); //read 1 or 0 from single pin
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); //entire port is 16 pins--hence uint16
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value); //Value is either 1 or 0--set or reset
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI); //for interrupt config; could also consider IRQGrouping param
void GPIO_IRQHandling(uint8_t PinNumber); //ISR--interrupt service routine aka interrupt handle for interrupt request (IRQ)

#endif /* INC_STM32L432XX_GPIO_DRIVER_H_ */
