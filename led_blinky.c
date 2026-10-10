/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *

INSTITUTO TECNOLOGICO DE LAS AMERICAS (ITLA)
Profesor: Wilkins Gabriel Cedano
Student: Joseph Ramirez
Matricula: 2021-0538
*/

#include "board.h"
#include "app.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "peripherals.h"
#include "stdlib.h"
#include "string.h"
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
uint8_t pwm_final = ((CTIMER0_PWM_PERIOD + 1 - CTIMER0_PWM_0_DUTY) * 100) / CTIMER0_PWM_PERIOD; // Valor inicial del PWM en porcentaje                                                                    // Buffer para almacenar la entrada de la consola
char input_dat[16] = {0};
int n = 0;                // Índice para el buffer de entrada
bool pwm_updated = false; // Bandera para indicar si el PWM ha sido actualizado
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
void LPUART0_ISRQHANDLER_RX(void)
{
    if ((kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)) != 0U)
    {
        uint8_t dat = LPUART_ReadByte((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR);

        if (dat >= '0' && dat <= '9') // Si es un dígito
        {
            if (n < (sizeof(input_dat) - 1))
            {
                input_dat[n++] = (char)dat;
                input_dat[n] = '\0';
                pwm_updated = true;
            }
        }
        else if (dat == '\r' || dat == '\n') // Si presionan Enter
        {
            // SOLO activamos la bandera si el búfer tiene contenido (evita enviar ceros por enters vacíos)
                pwm_updated = true;
        }
    }
}

void SW2_IRQHANDLER(void)
{ // Interrupt function for SW2
    uint8_t i = GPIO_GpioGetInterruptFlags(BOARD_INITPINS_SW_2_GPIO);
    if ((i & (7U << BOARD_INITPINS_SW_2_GPIO_PIN)) != 0UL)
    {
        PRINTF("SW2 Pressed\r\n");
    }
    GPIO_PortClearInterruptFlags(BOARD_INITPINS_SW_2_GPIO, 7U << BOARD_INITPINS_SW_2_GPIO_PIN);
}

void PWM_updater(void)
{
    // Interrupt function for PWM
    //  uint32_t j = 0;
    //  PRINTF("Ingrese el valor de PWM: ");
    //  SCANF("%d", &j);
    PRINTF("PWM update_ estamos en el bucle\r\n");
    //  CTIMER_UpdatePwmDutycycle(CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH, CTIMER0_PWM_0_CHANNEL, j);
}

int main(void)
{
    /* Board pin init */
    BOARD_InitHardware();
    GPIO_PinWrite(BOARD_INITPINS_LED_RED_GPIO, BOARD_INITPINS_LED_RED_GPIO_PIN, 0U); // Turn on the LED
    PRINTF("Iniciamos el programa\n");
    PRINTF("PWM_init: %d%%\r\n", pwm_final);

    while (1)
    {
        if (pwm_updated == false)
        {
            n = 0; // Reset the index for the next input
            if (pwm_final > 100)
                pwm_final = 100;

            PRINTF("PWM: %d%%\r\n", pwm_final);
            CTIMER_UpdatePwmDutycycle(CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH, CTIMER0_PWM_0_CHANNEL, pwm_final);
            SDK_DelayAtLeastUs(DELAY_TIME, SystemCoreClock);
        }
        else if (pwm_updated == true && input_dat[0] != '\0')
        {
            pwm_final = atoi(input_dat);             // Convierte el string a entero
            memset(input_dat, 0, sizeof(input_dat)); // Limpia el buffer de entrada
            pwm_updated = false; // Por seguridad, si llegara un enter vacío, limpia la bandera de inmediato
        }
        else {
            pwm_updated = false;
        }
    }
}