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
#define FREQ_500 999 / 2
#define FREQ_1000 999
#define FREQ_2000 999 * 2
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void STATE_MACHINE(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
typedef enum
{
    STATE_MANUAL, // Estado inicial, esperando entrada
    STATE_AUTO,   // Estado para actualizar el PWM
    STATE_PAUSE   // Estado de error, si es necesario
} state_t;

uint8_t pwm_final = ((CTIMER0_PWM_PERIOD + 1 - CTIMER0_PWM_0_DUTY) * 100) / CTIMER0_PWM_PERIOD; // Valor inicial del PWM en porcentaje                                                                    // Buffer para almacenar la entrada de la consola
uint32_t g_ms = STATE_MANUAL;                                                                   // Variable para almacenar el tiempo de tick de 1 segundo
char input_dat[64] = {0};
bool pwm_updated = false;                           // Bandera para indicar si el PWM ha sido actualizado
int n = STATE_MANUAL, current_state = STATE_MANUAL; // Índice para el buffer de entrada

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

void Tick_timer_CTIMER(uint32_t flags)
{
    if (flags & kCTIMER_Match0Flag)
    {
        g_ms++; // Incrementa tu base de tiempos global de 1 ms sin bloqueos
    }
}

void SW2_IRQHANDLER(void) // actualizar interrupt con doble flanco y debounce con condicionales de 30ms a 1.5s
{                         // Interrupt function for SW2
    uint8_t i = GPIO_GpioGetInterruptFlags(BOARD_INITPINS_SW_2_GPIO);
    if ((i & (7U << BOARD_INITPINS_SW_2_GPIO_PIN)) != 0UL)
    {
        PRINTF("SW2 Pressed\r\n");
    }
    GPIO_PortClearInterruptFlags(BOARD_INITPINS_SW_2_GPIO, 7U << BOARD_INITPINS_SW_2_GPIO_PIN);
}

void STATE_MACHINE(void)
{
    /* switch(current_state)
     {
         case STATE_MANUAL:
             //setear a 500Hz 25% duty cycle
             //CTIMER_SetupPwmPeriod(CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH, CTIMER0_PWM_0_CHANNEL, FREQ_500, 125, false);
             break;

         case STATE_AUTO:

             break;

         case STATE_PAUSE:
             // Aquí podrías manejar un estado de error o pausa si es necesario
             break;

         default:
             current_state = STATE_MANUAL; // Asegura que siempre haya un estado válido
             break;
     }*/
}

int main(void)
{
    uint32_t last_tick = g_ms; // Variable para almacenar el último tick de 1 segundo
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

            // delay_ms(1000); // Espera 1 segundo antes de permitir otra entrada
            if ((uint32_t)(g_ms - last_tick) >= 1000U)
                {
                    last_tick = g_ms; // Actualizamos la marca de referencia de forma limpia
                    PRINTF("PWM: %d%%\r\n", pwm_final);
                    CTIMER_UpdatePwmDutycycle(CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH, CTIMER0_PWM_0_CHANNEL, pwm_final);                  
                }
        }
        else if (pwm_updated == true && input_dat[0] != '\0')
        {
            pwm_final = atoi(input_dat);             // Convierte el string a entero
            memset(input_dat, 0, sizeof(input_dat)); // Limpia el buffer de entrada
            pwm_updated = false;                     // Por seguridad, si llegara un enter vacío, limpia la bandera de inmediato
        }
        else
        {
            pwm_updated = false;
        }
    }
}