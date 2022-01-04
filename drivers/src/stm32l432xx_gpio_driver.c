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
    if(EnorDi == ENABLE) {  //enable
        if(pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else  if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        }
    } else {                //disable
        if(pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        } else  if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        }
    }
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
    uint32_t temp = 0; //temp register
    //configure mode
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        //non-interrrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //position bits
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
        pGPIOHandle->pGPIOx->MODER |= temp; //set mode in actual register
    } else {
        //interrupt modes
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
            //configure FTSR (fall trigger selection register)
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            //clear corresponding RTSR bit to be safe
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
            //configure RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            //clear corresponding FTSR bit to be safe
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
            //configure both FTSR and RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        //2, configure GPIO port selection in SYSCFG_EXTI
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

        //3. enable the EXTI interrupt delivery using IMR (interrupt mask register)
        EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    }
    temp = 0; //reset temp

    //configure speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //position bits
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;
    //configure pupd settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //position bits
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;
    //configure output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //position bits
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;
    //configure alt functionality if req
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        //configure alt function registers
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF <<  (4 * temp2)); //clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<  (4 * temp2)); //setting alt func
    }
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
    if(pGPIOx == GPIOA) {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB) {
        GPIOB_REG_RESET();
    } else  if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOH) {
        GPIOH_REG_RESET();
    }
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
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); //shift pin desired to lsb then mask (cast to uint_8 to truncate?)
    return value;
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
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR; //return entire port
    return value;
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
    if (Value == GPIO_PIN_SET) {
        //write 1
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        //write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
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
    pGPIOx->ODR = Value; //assignment is fine here bc overwriting entire register
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
    pGPIOx->ODR ^= (1 << PinNumber); //XOR switches bit with 1, maintains og value elsewhere with 0
}

/*
 * IRQ Configuration and ISR handling
 */
/****************************************************************************
 * @fn              - GPIO_IRQInterruptConfig
 *
 * @brief           - Function for configuring interrupts on gpio peripheral, processor side(Cortex M4) config of NVIC
 *
 * @param[in]       - interrupt number
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDI)
{
    if (EnorDI == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            //Program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);

        } else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
        {
            //Program ISER1 register
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
        } else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            //program ISER2 register
            *NVIC_ISER2 |= (1 << IRQNumber % 64);
        }
    } else
    {
        if(IRQNumber <= 31)
        {
            //Program ISER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
        {
            //Program ISER1 register
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        } else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            //program ISER2 register
            *NVIC_ICER2 |= (1 << IRQNumber % 64);
        }
    }
}

/*
 * IRQ Priority Configuration and ISR handling
 */
/****************************************************************************
 * @fn              - GPIO_IRQPriorityConfig
 *
 * @brief           - Function for configuring interrupt priority on gpio peripheral, processor side(Cortex M4) config of NVIC
 *
 * @param[in]       - interrupt number
 * @param[in]       - interrupt priority
 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
   //1. Find out the IPR register
    uint8_t iprx = IRQNumber  / 4;
    uint8_t iprx_section = IRQNumber % 4;


    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx*4) |= ( IRQPriority << shift_amount);
}

/****************************************************************************
 * @fn              - GPIO_IRQHandling
 *
 * @brief           - Function for handling gpio interrupts
 *
 * @param[in]       - specific pin number

 *
 * @return          - none
 *
 * @Note            - none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    //clear the exti pr register corresponding to the pin number
    if (EXTI->PR & (1 << PinNumber)) //if set, it is pending and to be cleared
    {
        //clear
        EXTI->PR |= (1 << PinNumber); //write 1 to clear per ref manual
    }
}
