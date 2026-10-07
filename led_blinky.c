/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "app.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "peripherals.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DELAY_TIME 1000000U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
void SW2_IRQHANDLER(void){ // Interrupt function for SW2
    uint8_t i = GPIO_GpioGetInterruptFlags(BOARD_INITPINS_SW_2_GPIO);
    if(( i & (7U << BOARD_INITPINS_SW_2_GPIO_PIN)) != 0UL)
    {
        PRINTF("SW2 Pressed\r\n");
    }
    GPIO_PortClearInterruptFlags(BOARD_INITPINS_SW_2_GPIO, 7U << BOARD_INITPINS_SW_2_GPIO_PIN);
}

void PWM_updater(void){ // Interrupt function for PWM
    uint32_t j;
    PRINTF("Ingrese el valor de PWM: ");
    SCANF("%d", &j);
    PRINTF("PWM update\r\n");
    CTIMER_UpdatePwmDutycycle(CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH, CTIMER0_PWM_0_CHANNEL, j);
}

int main(void)
{
    /* Board pin init */
    BOARD_InitHardware();
    GPIO_PinWrite(BOARD_INITPINS_LED_RED_GPIO, BOARD_INITPINS_LED_RED_GPIO_PIN, 0U); // Turn on the LED 
    PRINTF("Iniciamos el programa\n");
    while (1)
    {
        PRINTF("Estamos dentro del bucle\n");
        SDK_DelayAtLeastUs(DELAY_TIME, SystemCoreClock);
        PWM_updater();
    }
}
