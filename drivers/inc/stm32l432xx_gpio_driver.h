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
    uint8_t GPIO_PinNumber; //pins range 0-15 so 1 byte (8 bits) enough
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
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
