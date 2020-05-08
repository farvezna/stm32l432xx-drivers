/*
 * stm32l432xx_gpio_driver.c
 *
 *  Created on: May 8, 2020
 *      Author: farve
 */


#include "stm32l432xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/****************************************************************************
 * @fn              - GPiO_PeriClockControl
 *
 * @brief           - Function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]       - base address of the gpio peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

}

/*
 * Init and De-init
 */
/****************************************************************************
 * @fn              - GPIO_Init
 *
 * @brief           - Initialize the gpio
 *
 * @param[in]       - Handler for gpio pin
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}

/****************************************************************************
 * @fn              - GPIO_DeInit
 *
 * @brief           - Deinitialize the gpio
 *
 * @param[in]       - base address of gpio peripheral
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*
 * Data read/write
 */

/****************************************************************************
 * @fn              - GPIO_ReadFromInputPin
 *
 * @brief           - Read data from single gpio input pin
 *
 * @param[in]       - base address of gpio peripheral
 * @param[in]       - specific pin number on gpio port
 *
 * @return          - either 1(HIGH) or 0 (LOW)
 *
 * @Note            - none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/****************************************************************************
 * @fn              - GPIO_ReadFromInputPort
 *
 * @brief           - read data from entire set of pins on gpio port
 *
 * @param[in]       - base address of the gpio peripheral
 *
 * @return          - 16 bits of 1(HIGH) or 0(LOW) for each of 16 pins on gpio port
 *
 * @Note            - none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

/****************************************************************************
 * @fn              - GPIO_WriteToOutputPin
 *
 * @brief           - Write data specfic gpio output pin
 *
 * @param[in]       - base address of gpio peripheral
 * @param[in]       - specfici pin number on gpio port
 * @param[in]       - specific value--SET or RESET
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

/****************************************************************************
 * @fn              - GPIO_WriteToOutputPort
 *
 * @brief           - Write data to entire gpio port
 *
 * @param[in]       - base address of gpio peripheral
 * @param[in]       - 16 bits of value to be written to gpio port
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/****************************************************************************
 * @fn              - GPIO_ToggleOutputPin
 *
 * @brief           - Function to toggle value of specfic gpio pin
 *
 * @param[in]       - base address of gpio peripheral
 * @param[in]       - specific pin number on gpio port
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR handling
 */
/****************************************************************************
 * @fn              - GPIO_IRQConfig
 *
 * @brief           - Function for configuring interrupts on gpio peripheral
 *
 * @param[in]       - interrupt number
 * @param[in]       - interrupt priority
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI)
{

}

/****************************************************************************
 * @fn              - GPIO_IRQHandling
 *
 * @brief           - Function for handling gpio interrupts
 *
 * @param[in]       - specific pin numberr

 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
