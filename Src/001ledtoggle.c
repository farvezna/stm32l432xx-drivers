/*
 * 001ledtoggle.c
 *
 *  Created on: May 9, 2020
 *      Author: farve
 */

#include "stm32l432xx.h"

void delay(void)
{
    for(uint32_t i = 0; i < 50000; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed;

    //setup configurations
    GpioLed.pGPIOx = GPIOB;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //since already push pull type (vs open drain)

    GPIO_PeriClockControl(GPIOB, ENABLE); //must enable clock before initializing hardware
    GPIO_Init(&GpioLed);

    while(1) {
        GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_3);
        delay();
    }
    return 0;
}
