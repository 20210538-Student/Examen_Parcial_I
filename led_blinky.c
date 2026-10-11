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
void BUTTON_STATE_MACHINE(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
typedef enum
{
    BTN_IDLE,
    BTN_DEBOUNCING_PRESS,
    BTN_PRESSED,
    BTN_DEBOUNCING_RELEASE
} button_state_t;

button_state_t btn_state = BTN_IDLE;
uint32_t btn_timer = 0;
bool long_press_dispatched = false;

typedef enum
{
    STATE_MANUAL, // Estado inicial, esperando entrada
    STATE_AUTO,   // Estado para actualizar el PWM
    STATE_PAUSE   // Estado de error, si es necesario
} state_t;

static volatile bool g_button_edge_pending = false;
static volatile uint8_t pwm_final = ((CTIMER0_PWM_PERIOD + 1 - CTIMER0_PWM_0_DUTY) * 100) / CTIMER0_PWM_PERIOD; // Valor inicial del PWM en porcentaje                                                                    // Buffer para almacenar la entrada de la consola
volatile uint32_t g_ms = STATE_MANUAL;                                                                   // Variable para almacenar el tiempo de tick de 1 segundo
volatile char input_dat[64] = {0};
volatile bool pwm_updated = false;                  // Bandera para indicar si el PWM ha sido actualizado
volatile uint32_t last_rx_ms = 0;
int n = STATE_MANUAL, current_state = STATE_MANUAL; // Índice para el buffer de entrada

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
void LPUART0_ISRQHANDLER_RX(void)
{
    LPUART_Type *base = (LPUART_Type *)BOARD_DEBUG_UART_BASEADDR;
    uint32_t flags = LPUART_GetStatusFlags(base);

    if (flags & kLPUART_RxOverrunFlag)
    {
        LPUART_ClearStatusFlags(base, kLPUART_RxOverrunFlag);
    }

    // Leer TODO lo que haya en el FIFO, no solo un byte
    while (LPUART_GetRxFifoCount(base) > 0U)
    {
        uint8_t dat = LPUART_ReadByte(base);

        if (!pwm_updated)
        {
            if (dat >= '0' && dat <= '9')
            {
                if (n < (int)(sizeof(input_dat) - 1))
                {
                    input_dat[n++] = (char)dat;
                    input_dat[n] = '\0';
                    last_rx_ms = g_ms;
                }
            }
            else if ((dat == '\r' || dat == '\n') && n > 0)
            {
                pwm_updated = true;   // CR, LF o CRLF
            }
        }
    }

    SDK_ISR_EXIT_BARRIER;
}

void Tick_timer_CTIMER(uint32_t flags)
{
    if (flags & kCTIMER_Match0Flag)
    {
        g_ms++; // Incrementa tu base de tiempos global de 1 ms sin bloqueos
    }
}

void SW2_IRQHANDLER(void)
{
    // 1. Obtiene las banderas de interrupción del puerto del botón
    uint32_t int_flags = GPIO_GpioGetInterruptFlags(BOARD_INITPINS_SW_2_GPIO);
    uint32_t pin_mask = (1UL << BOARD_INITPINS_SW_2_GPIO_PIN);

    if ((int_flags & pin_mask) != 0U)
    {
        // 2. Limpia la bandera de interrupción inmediatamente
        GPIO_GpioClearInterruptFlags(BOARD_INITPINS_SW_2_GPIO, pin_mask);

        // 3. Notifica al bucle principal de forma segura
        g_button_edge_pending = true;
    }
    SDK_ISR_EXIT_BARRIER; // Asegura el cierre correcto de la ISR en arquitecturas Cortex-M
}

void BUTTON_STATE_MACHINE(void)
{
    uint32_t now = g_ms;

    // --- 1. GESTIÓN DEL BOTÓN Y ANTIRREBOTE (30 ms) ---
    switch (btn_state)
    {
    case BTN_IDLE:
        if (g_button_edge_pending)
        {
            g_button_edge_pending = false;
            btn_timer = now; // Inicia la ventana de antirrebote
            btn_state = BTN_DEBOUNCING_PRESS;
        }
        break;

    case BTN_DEBOUNCING_PRESS:
        // Espera a que transcurran 30 ms de nivel estable
        if ((uint32_t)(now - btn_timer) >= 30U)
        {
            // Lee el estado físico real del pin (asumiendo lógica invertida Pull-Up: 0 = presionado)
            uint32_t pin_val = GPIO_PinRead(BOARD_INITPINS_SW_2_GPIO, BOARD_INITPINS_SW_2_GPIO_PIN);

            if (pin_val == 0U) // Se confirmó que está presionado
            {
                btn_timer = now; // Reinicia el cronómetro para medir la duración de la pulsación
                long_press_dispatched = false;
                btn_state = BTN_PRESSED;
            }
            else
            {
                // Fue un rebote falso, regresa a reposo
                btn_state = BTN_IDLE;
            }
        }
        break;

    case BTN_PRESSED:
    {
        uint32_t press_duration = (uint32_t)(now - btn_timer);
        uint32_t pin_val = GPIO_PinRead(BOARD_INITPINS_SW_2_GPIO, BOARD_INITPINS_SW_2_GPIO_PIN);

        // Si el usuario soltó el botón antes de los 1500 ms -> Pulsación Corta
        if (pin_val != 0U)
        {
            btn_timer = now;
            btn_state = BTN_DEBOUNCING_RELEASE;
        }
        // Si mantiene presionado y alcanza o supera los 1500 ms (y no se ha disparado) -> Pulsación Larga
        else if (press_duration >= 1500U && !long_press_dispatched)
        {
            long_press_dispatched = true;

            // ---> INSTRUCCIÓN Y (Pulsación Larga: Entra/Sale de PAUSA) <---
            PRINTF("Acción Y: Pulsación Larga detectada (>= 1500ms)\r\n");
        }
    }
    break;

    case BTN_DEBOUNCING_RELEASE:
        // Ventana de antirrebote al soltar (30 ms)
        if ((uint32_t)(now - btn_timer) >= 30U)
        {
            uint32_t pin_val = GPIO_PinRead(BOARD_INITPINS_SW_2_GPIO, BOARD_INITPINS_SW_2_GPIO_PIN);
            if (pin_val != 0U) // Confirmado liberado
            {
                // Si nunca se disparó la pulsación larga, significa que fue una pulsación corta (< 1500ms)
                if (!long_press_dispatched)
                {
                    // ---> INSTRUCCIÓN X (Pulsación Corta: Cambia de estado / Modo) <---
                    PRINTF("Acción X: Pulsación Corta detectada (< 1500ms)\r\n");
                }
            }
            btn_state = BTN_IDLE;
        }
        break;
    }
}

int main(void)
{
    uint32_t last_tick = g_ms;
    
    /* Board pin init */
    BOARD_InitHardware();
    LPUART_SetRxFifoWatermark((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR, 0U);
    GPIO_PinWrite(BOARD_INITPINS_LED_RED_GPIO, BOARD_INITPINS_LED_RED_GPIO_PIN, 0U); // Encender LED
    
    PRINTF("Iniciamos el programa\r\n");
    PRINTF("PWM_init: %d%%\r\n", pwm_final);

    CTIMER_UpdatePwmDutycycle(CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH, CTIMER0_PWM_0_CHANNEL, pwm_final);

    while (1)
    {
        // 1. Botón (antirrebote y flancos)
        BUTTON_STATE_MACHINE();

        // 2. Procesar línea UART cuando la ISR detectó Enter
        if (pwm_updated)
        {
            char line[sizeof(input_dat)];

            uint32_t primask = DisableGlobalIRQ();
            memcpy(line, (const void *)input_dat, sizeof(line));
            memset((void *)input_dat, 0, sizeof(input_dat));
            n = 0;
            pwm_updated = false;
            EnableGlobalIRQ(primask);

            if (line[0] != '\0')
            {
                uint32_t val = (uint32_t)atoi(line);
                if (val > 100U)
                {
                    val = 100U;
                }
                pwm_final = (uint8_t)val;

                PRINTF("PWM actualizado: %d%%\r\n", pwm_final);
                CTIMER_UpdatePwmDutycycle(CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH, CTIMER0_PWM_0_CHANNEL, pwm_final);
            }
        }

        // 3. Timeout de entrada incompleta (5 s sin recibir dígitos)
        {
            uint32_t primask = DisableGlobalIRQ();
            int      n_snap  = n;
            uint32_t rx_snap = last_rx_ms;
            bool     ready   = pwm_updated;
            uint32_t now     = g_ms;
            EnableGlobalIRQ(primask);

            if (n_snap > 0 && !ready && (uint32_t)(now - rx_snap) >= 5000U)
            {
                primask = DisableGlobalIRQ();
                // Revalida: la ISR pudo recibir otro dígito entre el snapshot y ahora
                if (n > 0 && !pwm_updated && (uint32_t)(g_ms - last_rx_ms) >= 5000U)
                {
                    memset((void *)input_dat, 0, sizeof(input_dat));
                    n = 0;
                    EnableGlobalIRQ(primask);
                    PRINTF("Timeout: entrada descartada\r\n");
                }
                else
                {
                    EnableGlobalIRQ(primask);
                }
            }
        }

        // 4. Tarea periódica de 1 segundo
        if ((uint32_t)(g_ms - last_tick) >= 1000U)
        {
            last_tick += 1000U;
            // PRINTF("Estado actual PWM: %d%%\r\n", pwm_final);
        }
    }
}