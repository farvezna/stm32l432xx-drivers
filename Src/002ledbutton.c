/*
 * 002ledbutton.c
 *
 *  Created on: May 18, 2020
 *      Author: farve
 */

#include "stm32l432xx.h"

#define HIGH            1
#define BTN_PRESSED     HIGH

void delay(void)
{
    for(uint32_t i = 0; i < 50000; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed, GpioBtn;

    //setup configurations
    GpioLed.pGPIOx = GPIOB;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //since already push pull type (vs open drain)

    GPIO_PeriClockControl(GPIOB, ENABLE); //must enable clock before initializing hardware
    GPIO_Init(&GpioLed);

    GpioBtn.pGPIOx = GPIOA;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //using external resistor

    GPIO_PeriClockControl(GPIOA, ENABLE); //must enable clock before initializing hardware
    GPIO_Init(&GpioBtn);

    while(1) {
        if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_7) == 0) { //just to be explicit using macro for now
            delay(); //debouncing
            GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_3);
        }
    }
    return 0;
}
