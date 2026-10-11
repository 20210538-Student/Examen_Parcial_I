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
#include "fsl_ctimer.h"
#include "peripherals.h"
#include "stdlib.h"
#include "string.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DELAY_TIME 1000000U

/* Frecuencias PWM permitidas (Hz). Antes eran valores de periodo (y estaban cruzados);
 * ahora el periodo se calcula con el reloj real del CTIMER en Pwm_Configure(). */
#define FREQ_500  500U
#define FREQ_1000 1000U
#define FREQ_2000 2000U

/* Comportamiento requerido */
#define PWM_FREQ_INITIAL   FREQ_1000 /* MANUAL inicial: 1 kHz           */
#define PWM_DUTY_INITIAL   25U       /* MANUAL inicial: duty 25 %       */
#define AUTO_DUTY_MIN      10U       /* AUTO: rampa triangular 10 % ... */
#define AUTO_DUTY_MAX      90U       /*                        ... 90 % */
#define AUTO_STEP_MS       20U       /* 1 punto porcentual cada 20 ms   */
#define LED_PERIOD_AUTO_MS 500U      /* LED en AUTO: alterna cada 500 ms */
#define LED_PERIOD_PAUSE_MS 125U     /* LED en PAUSA: alterna cada 125 ms */
#define BTN_DEBOUNCE_MS    30U
#define BTN_LONG_MS        1500U

/* PWM: periferico/canales generados por Config Tools */
#define PWM_CTIMER    CTIMER0_PERIPHERAL
#define PWM_PERIOD_CH CTIMER0_PWM_PERIOD_CH
#define PWM_DUTY_CH   CTIMER0_PWM_0_CHANNEL
/* Reloj de entrada del CTIMER del PWM (si tu SDK lo llama distinto, cambia solo esta linea) */
#ifndef PWM_SRC_CLOCK_HZ
#define PWM_SRC_CLOCK_HZ CLOCK_GetCTimerClkFreq(0U)
#endif

/* LED de estado (activo en bajo). IMPORTANTE: el LED rojo de la placa es P3_12, el mismo pin que el
 * mandato asigna al PWM (J1-15). Si tu salida PWM esta en P3_12, configura LED_GREEN (P3_13) en el
 * Pins Tool y cambia SOLO estas dos lineas por BOARD_INITPINS_LED_GREEN_GPIO / _GPIO_PIN. */
#define STATUS_LED_GPIO BOARD_INITPINS_LED_RED_GPIO
#define STATUS_LED_PIN  BOARD_INITPINS_LED_RED_GPIO_PIN

/* Consola UART */
#define RX_RING_SIZE     256U /* potencia de 2, >= 128 */
#define RX_RING_MASK     (RX_RING_SIZE - 1U)
#define LINE_MAX_CHARS   63U /* 63 caracteres + NUL */
#define MAX_TOKENS       2U

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
uint32_t btn_timer = 0;          // Instante del ultimo flanco (ventana de antirrebote)
uint32_t btn_press_ms = 0;       // Instante del flanco de la pulsacion ya validada
bool long_press_dispatched = false;

typedef enum
{
    STATE_MANUAL, // Estado inicial: PWM 1 kHz, duty manual, LED fijo
    STATE_AUTO,   // Rampa triangular 10-90 %, LED 500 ms
    STATE_PAUSE   // PWM 0 %, LED 125 ms
} state_t;

typedef enum
{
    ERR_COMMAND,
    ERR_ARG,
    ERR_RANGE,
    ERR_STATE,
    ERR_OVERFLOW,
    ERR_COUNT
} err_t;

typedef enum
{
    NUM_OK,
    NUM_ERR_ARG,
    NUM_ERR_RANGE
} num_result_t;

static volatile bool g_button_edge_pending = false;
static volatile uint32_t g_button_edge_ms = 0;  // Instante del ultimo flanco (lo escribe la ISR)
static volatile uint8_t pwm_final = PWM_DUTY_INITIAL; // Duty que realmente sale por el pin (0 en PAUSA)
volatile uint32_t g_ms = 0;                     // Base de tiempo de 1 ms (CTIMER). Nunca se reinicia.

state_t current_state = STATE_MANUAL;           // Estado actual
static state_t g_resume_state = STATE_MANUAL;   // Estado guardado al entrar a PAUSA
static uint8_t g_manual_duty = PWM_DUTY_INITIAL;// Ultimo duty manual
static uint8_t g_auto_duty = AUTO_DUTY_MIN;     // Valor de la rampa AUTO
static bool g_auto_up = true;                   // Direccion de la rampa
static uint32_t g_auto_last_ms = 0;
static uint32_t g_freq_hz = PWM_FREQ_INITIAL;
static bool g_stream_on = false;
static uint32_t g_tel_last_ms = 0;
static uint32_t g_uptime_s = 0;

/* LED de estado */
static bool led_on = false;
static uint32_t led_last_ms = 0;

/* Informacion del PWM (documentacion: reloj efectivo, prescaler, registro de periodo) */
static uint32_t g_pwm_clk_hz = 0;
static uint32_t g_pwm_pr = 0;
static uint32_t g_pwm_timer_hz = 0;
static uint32_t g_pwm_period_reg = 0;

/* Contadores */
static uint32_t g_btn_short_cnt = 0;   // cortas aceptadas
static uint32_t g_btn_long_cnt = 0;    // largas aceptadas
static uint32_t g_btn_short_ignored = 0; // cortas aceptadas pero ignoradas por estar en PAUSA
static uint32_t g_ok_cnt = 0;
static uint32_t g_err_cnt[ERR_COUNT] = {0};
static uint32_t g_rx_loss_events = 0;

/* Buffer circular de recepcion (productor: ISR, consumidor: main).
 * Los indices son contadores absolutos; la posicion real es (indice & MASK). */
static volatile uint8_t g_rx_ring[RX_RING_SIZE];
static volatile uint32_t g_rx_head = 0;
static volatile uint32_t g_rx_tail = 0;
static volatile bool g_rx_loss_pending = false; // se perdieron datos en la posicion g_rx_loss_pos
static volatile uint32_t g_rx_loss_pos = 0;

/* Ensamblado de linea (solo main) */
static char g_line[LINE_MAX_CHARS + 1U];
static uint32_t g_line_len = 0;
static bool g_line_discard = false;

static const char *const ERR_TEXT[ERR_COUNT] = {"ERR COMMAND\r\n", "ERR ARG\r\n", "ERR RANGE\r\n", "ERR STATE\r\n",
                                                 "ERR OVERFLOW\r\n"};

static const char HELP_TEXT[] = "OK\r\n"
                                "HELP\r\n"
                                "STATUS\r\n"
                                "MODE MANUAL|AUTO\r\n"
                                "DUTY n (MANUAL, 0-100)\r\n"
                                "FREQ n (500|1000|2000)\r\n"
                                "PAUSE\r\n"
                                "RESUME\r\n"
                                "STREAM ON|OFF\r\n";

/*******************************************************************************
 * LED de estado (se maneja en main, nunca dentro de una ISR)
 ******************************************************************************/
static void LED_Set(bool on)
{
    led_on = on;
    GPIO_PinWrite(STATUS_LED_GPIO, STATUS_LED_PIN, on ? 0U : 1U); // activo en bajo
}

/* Se llama en cada cambio de estado: el patron arranca encendido y con fase nueva */
static void LED_Restart(void)
{
    led_last_ms = g_ms;
    LED_Set(true);
}

static void LED_Task(void)
{
    uint32_t half;

    if (current_state == STATE_MANUAL)
    {
        if (!led_on)
        {
            LED_Set(true); // encendido fijo
        }
        return;
    }

    half = (current_state == STATE_AUTO) ? LED_PERIOD_AUTO_MS : LED_PERIOD_PAUSE_MS;

    if ((uint32_t)(g_ms - led_last_ms) >= (2U * half))
    {
        led_last_ms = g_ms; // main se retraso mucho: resincroniza en vez de parpadear en rafaga
    }
    if ((uint32_t)(g_ms - led_last_ms) >= half)
    {
        led_last_ms += half;
        LED_Set(!led_on);
    }
}

/*******************************************************************************
 * PWM por CTIMER (frecuencia y duty)
 *
 *  f_timer     = f_clk_CTIMER / (PR + 1)
 *  MR[PERIODO] = round(f_timer / f_pwm) - 1
 *  MR[DUTY]    = ((MR[PERIODO] + 1) * (100 - duty)) / 100
 *  Salida: BAJA hasta MR[DUTY], ALTA hasta el fin del ciclo (igual que el driver del SDK).
 *
 *  Tratamiento especial (el driver calcula 0 % como MR[PERIODO], lo que deja un pulso de 1 tick):
 *    duty = 0   -> MR[DUTY] = MR[PERIODO] + 1 : nunca coincide, salida baja constante
 *    duty = 100 -> MR[DUTY] = 0                : salida alta constante
 *  El tick de 1 ms esta en otro CTIMER, por eso cambiar la frecuencia no lo altera.
 ******************************************************************************/
static uint32_t Pwm_MatchFromDuty(uint32_t period_reg, uint32_t duty)
{
    if (duty == 0U)
    {
        return period_reg + 1U;
    }
    if (duty >= 100U)
    {
        return 0U;
    }
    return ((period_reg + 1U) * (100U - duty)) / 100U;
}

/* Cambia solo el duty. Se escribe MR (efecto inmediato) y MSR (shadow) con el mismo valor. */
static void Pwm_SetDuty(uint8_t duty)
{
    uint32_t match = Pwm_MatchFromDuty(g_pwm_period_reg, duty);

    PWM_CTIMER->MR[PWM_DUTY_CH] = match;
    CTIMER_UpdatePwmPulsePeriod(PWM_CTIMER, PWM_DUTY_CH, match); // escribe MSR
}

/* Cambia frecuencia y duty. Se detiene/reinicia solo el contador del PWM para que un periodo
 * nuevo mas corto no deje el contador "pasado" (esperaria al desborde de 32 bits). */
static void Pwm_Configure(uint32_t freq_hz, uint8_t duty)
{
    g_pwm_clk_hz = (uint32_t)PWM_SRC_CLOCK_HZ;
    g_pwm_pr = PWM_CTIMER->PR & CTIMER_PR_PRVAL_MASK;
    g_pwm_timer_hz = g_pwm_clk_hz / (g_pwm_pr + 1U);

    uint32_t ticks = (g_pwm_timer_hz + (freq_hz / 2U)) / freq_hz;
    if (ticks < 2U)
    {
        ticks = 2U;
    }
    uint32_t period = ticks - 1U;
    uint32_t match = Pwm_MatchFromDuty(period, duty);

    CTIMER_StopTimer(PWM_CTIMER);
    PWM_CTIMER->MR[PWM_PERIOD_CH] = period;
    CTIMER_UpdatePwmPulsePeriod(PWM_CTIMER, PWM_PERIOD_CH, period); // MSR del periodo
    PWM_CTIMER->MR[PWM_DUTY_CH] = match;
    CTIMER_UpdatePwmPulsePeriod(PWM_CTIMER, PWM_DUTY_CH, match);    // MSR del duty
    CTIMER_Reset(PWM_CTIMER);
    CTIMER_StartTimer(PWM_CTIMER);

    g_pwm_period_reg = period;
    g_freq_hz = freq_hz;
}

/*******************************************************************************
 * Maquina de estados MANUAL / AUTO / PAUSA (la usan botones y comandos por igual)
 ******************************************************************************/
static const char *State_Name(state_t s)
{
    switch (s)
    {
    case STATE_MANUAL:
        return "MANUAL";
    case STATE_AUTO:
        return "AUTO";
    default:
        return "PAUSE";
    }
}

static void Mode_EnterManual(void)
{
    current_state = STATE_MANUAL;
    pwm_final = g_manual_duty; // restaura el ultimo duty manual
    Pwm_SetDuty(pwm_final);
    LED_Restart();
}

static void Mode_EnterAuto(void)
{
    current_state = STATE_AUTO;
    g_auto_duty = AUTO_DUTY_MIN; // AUTO siempre inicia en 10 % y asciende
    g_auto_up = true;
    g_auto_last_ms = g_ms;
    pwm_final = g_auto_duty;
    Pwm_SetDuty(pwm_final);
    LED_Restart();
}

/* PAUSA: salida a 0 %; modo, duty manual, valor y direccion de la rampa se conservan. */
static void Pause_Enter(void)
{
    if (current_state != STATE_PAUSE)
    {
        g_resume_state = current_state;
        current_state = STATE_PAUSE;
        pwm_final = 0U;
        Pwm_SetDuty(0U);
        LED_Restart();
    }
}

static bool Pause_Exit(void)
{
    if (current_state != STATE_PAUSE)
    {
        return false;
    }
    current_state = g_resume_state;
    pwm_final = (current_state == STATE_AUTO) ? g_auto_duty : g_manual_duty;
    Pwm_SetDuty(pwm_final);
    g_auto_last_ms = g_ms; // continua desde los mismos valores, sin recuperar pasos del tiempo pausado
    LED_Restart();
    return true;
}

/* Rampa triangular 10 % <-> 90 %, 1 punto cada 20 ms: cada tramo dura 80 x 20 ms = 1,6 s */
static void Auto_Task(void)
{
    if ((current_state != STATE_AUTO) || ((uint32_t)(g_ms - g_auto_last_ms) < AUTO_STEP_MS))
    {
        return;
    }
    g_auto_last_ms += AUTO_STEP_MS;

    if (g_auto_up)
    {
        g_auto_duty++;
        if (g_auto_duty >= AUTO_DUTY_MAX)
        {
            g_auto_duty = AUTO_DUTY_MAX;
            g_auto_up = false;
        }
    }
    else
    {
        g_auto_duty--;
        if (g_auto_duty <= AUTO_DUTY_MIN)
        {
            g_auto_duty = AUTO_DUTY_MIN;
            g_auto_up = true;
        }
    }
    pwm_final = g_auto_duty;
    Pwm_SetDuty(pwm_final);
}

/*******************************************************************************
 * Comandos UART
 ******************************************************************************/
static void Reply_OK(void)
{
    g_ok_cnt++;
    PRINTF("OK\r\n");
}

static void Reply_Err(err_t e)
{
    g_err_cnt[e]++;
    PRINTF("%s", ERR_TEXT[e]);
}

/* Valida TODA la cadena: solo digitos. "-n" es un entero valido fuera de rango -> RANGE.
 * Cualquier otro caracter -> ARG. Desbordamiento de uint32 -> RANGE. (No se usa atoi.) */
static num_result_t Parse_UInt(const char *s, uint32_t *out)
{
    bool neg = false;
    bool huge = false;
    uint32_t v = 0U;

    if (*s == '-')
    {
        neg = true;
        s++;
    }
    if (*s == '\0')
    {
        return NUM_ERR_ARG;
    }
    for (; *s != '\0'; s++)
    {
        if ((*s < '0') || (*s > '9'))
        {
            return NUM_ERR_ARG;
        }
        uint32_t d = (uint32_t)(*s - '0');
        if (!huge)
        {
            if (v > ((UINT32_MAX - d) / 10U))
            {
                huge = true;
            }
            else
            {
                v = (v * 10U) + d;
            }
        }
    }
    if (neg || huge)
    {
        return NUM_ERR_RANGE;
    }
    *out = v;
    return NUM_OK;
}

static void Cmd_Status(void)
{
    g_ok_cnt++;
    PRINTF("OK MODE=%s", State_Name(current_state));
    if (current_state == STATE_PAUSE)
    {
        PRINTF(" RESUME_TO=%s", State_Name(g_resume_state));
    }
    PRINTF(" FREQ=%u DUTY=%u MANUAL_DUTY=%u UPTIME=%u STREAM=%s", (unsigned)g_freq_hz, (unsigned)pwm_final,
           (unsigned)g_manual_duty, (unsigned)g_uptime_s, g_stream_on ? "ON" : "OFF");
    PRINTF(" BTN_SHORT=%u BTN_LONG=%u CMD_OK=%u", (unsigned)g_btn_short_cnt, (unsigned)g_btn_long_cnt,
           (unsigned)g_ok_cnt);
    PRINTF(" ERR_COMMAND=%u ERR_ARG=%u ERR_RANGE=%u ERR_STATE=%u ERR_OVERFLOW=%u RX_LOSS=%u\r\n",
           (unsigned)g_err_cnt[ERR_COMMAND], (unsigned)g_err_cnt[ERR_ARG], (unsigned)g_err_cnt[ERR_RANGE],
           (unsigned)g_err_cnt[ERR_STATE], (unsigned)g_err_cnt[ERR_OVERFLOW], (unsigned)g_rx_loss_events);
}

/* Orden de validacion: comando conocido -> estado -> argumentos -> rango.
 * Ningun rechazo modifica parametros: todo se valida antes de aplicar. */
static void Cmd_Execute(char *line)
{
    char *tok[MAX_TOKENS];
    uint32_t nt = 0U;
    char *p = line;

    /* Tokeniza en sitio; separadores: espacio/tab (se ignoran al inicio y al final) */
    while (*p != '\0')
    {
        while ((*p == ' ') || (*p == '\t'))
        {
            *p = '\0';
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        if (nt < MAX_TOKENS)
        {
            tok[nt] = p;
        }
        nt++;
        while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
        {
            p++;
        }
    }

    if (nt == 0U)
    {
        return; // linea solo con espacios: se ignora
    }

    const char *cmd = tok[0];

    if (strcmp(cmd, "HELP") == 0)
    {
        if (nt != 1U)
        {
            Reply_Err(ERR_ARG);
        }
        else
        {
            g_ok_cnt++;
            PRINTF("%s", HELP_TEXT);
        }
    }
    else if (strcmp(cmd, "STATUS") == 0)
    {
        if (nt != 1U)
        {
            Reply_Err(ERR_ARG);
        }
        else
        {
            Cmd_Status();
        }
    }
    else if (strcmp(cmd, "PAUSE") == 0)
    {
        if (nt != 1U)
        {
            Reply_Err(ERR_ARG);
        }
        else
        {
            Pause_Enter(); // si ya estaba en PAUSA no cambia el estado guardado
            Reply_OK();
        }
    }
    else if (strcmp(cmd, "RESUME") == 0)
    {
        if (nt != 1U)
        {
            Reply_Err(ERR_ARG);
        }
        else if (!Pause_Exit())
        {
            Reply_Err(ERR_STATE); // fuera de PAUSA
        }
        else
        {
            Reply_OK();
        }
    }
    else if (strcmp(cmd, "MODE") == 0)
    {
        if (current_state == STATE_PAUSE)
        {
            Reply_Err(ERR_STATE);
        }
        else if (nt != 2U)
        {
            Reply_Err(ERR_ARG);
        }
        else if (strcmp(tok[1], "MANUAL") == 0)
        {
            Mode_EnterManual();
            Reply_OK();
        }
        else if (strcmp(tok[1], "AUTO") == 0)
        {
            Mode_EnterAuto();
            Reply_OK();
        }
        else
        {
            Reply_Err(ERR_ARG);
        }
    }
    else if (strcmp(cmd, "DUTY") == 0)
    {
        uint32_t v = 0U;

        if (current_state != STATE_MANUAL) // AUTO y PAUSA lo rechazan
        {
            Reply_Err(ERR_STATE);
        }
        else if (nt != 2U)
        {
            Reply_Err(ERR_ARG);
        }
        else
        {
            num_result_t r = Parse_UInt(tok[1], &v);
            if (r == NUM_ERR_ARG)
            {
                Reply_Err(ERR_ARG);
            }
            else if ((r == NUM_ERR_RANGE) || (v > 100U))
            {
                Reply_Err(ERR_RANGE);
            }
            else
            {
                g_manual_duty = (uint8_t)v;
                pwm_final = (uint8_t)v;
                Pwm_SetDuty(pwm_final);
                Reply_OK();
            }
        }
    }
    else if (strcmp(cmd, "FREQ") == 0)
    {
        uint32_t v = 0U;

        if (current_state == STATE_PAUSE)
        {
            Reply_Err(ERR_STATE);
        }
        else if (nt != 2U)
        {
            Reply_Err(ERR_ARG);
        }
        else
        {
            num_result_t r = Parse_UInt(tok[1], &v);
            if (r == NUM_ERR_ARG)
            {
                Reply_Err(ERR_ARG);
            }
            else if ((r == NUM_ERR_RANGE) || ((v != FREQ_500) && (v != FREQ_1000) && (v != FREQ_2000)))
            {
                Reply_Err(ERR_RANGE);
            }
            else
            {
                Pwm_Configure(v, pwm_final); // conserva el duty aplicado
                Reply_OK();
            }
        }
    }
    else if (strcmp(cmd, "STREAM") == 0)
    {
        if (nt != 2U)
        {
            Reply_Err(ERR_ARG);
        }
        else if (strcmp(tok[1], "ON") == 0)
        {
            if (!g_stream_on)
            {
                g_stream_on = true;
                g_tel_last_ms = g_ms;
            }
            Reply_OK();
        }
        else if (strcmp(tok[1], "OFF") == 0)
        {
            g_stream_on = false;
            Reply_OK();
        }
        else
        {
            Reply_Err(ERR_ARG);
        }
    }
    else
    {
        Reply_Err(ERR_COMMAND);
    }
}

/*******************************************************************************
 * UART: recepcion (ISR -> buffer circular -> main)
 ******************************************************************************/
/* Marca que se perdieron datos justo en la posicion actual del buffer (solo desde la ISR). */
static inline void Rx_MarkLoss(void)
{
    if (!g_rx_loss_pending)
    {
        g_rx_loss_pos = g_rx_head;
        g_rx_loss_pending = true;
    }
}

static inline void Rx_Push(uint8_t c)
{
    if ((g_rx_head - g_rx_tail) >= RX_RING_SIZE)
    {
        Rx_MarkLoss(); // buffer lleno: el byte se descarta y se registra
        return;
    }
    g_rx_ring[g_rx_head & RX_RING_MASK] = c;
    g_rx_head++;
}

static void Console_ProcessByte(uint8_t c)
{
    if ((c == '\r') || (c == '\n')) // CR, LF o CRLF
    {
        if (g_line_discard)
        {
            // la orden afectada se descarta completa y se resincroniza aqui
            g_line_discard = false;
            g_line_len = 0U;
            Reply_Err(ERR_OVERFLOW);
        }
        else if (g_line_len > 0U)
        {
            g_line[g_line_len] = '\0';
            g_line_len = 0U;
            Cmd_Execute(g_line);
        }
        // linea vacia: se ignora (cubre el 2do byte de CRLF -> una sola orden)
        return;
    }

    if (g_line_discard)
    {
        return;
    }
    if (g_line_len < LINE_MAX_CHARS)
    {
        g_line[g_line_len++] = (char)c;
    }
    else
    {
        // 64.o caracter: desbordamiento de linea, no se ejecuta ningun fragmento
        g_line_discard = true;
        g_line_len = 0U;
    }
}

static void Console_Poll(void)
{
    for (;;)
    {
        // Perdida de datos: se aplica exactamente al llegar a la posicion donde ocurrio
        uint32_t primask = DisableGlobalIRQ();
        bool loss_here = g_rx_loss_pending && (g_rx_tail == g_rx_loss_pos);
        if (loss_here)
        {
            g_rx_loss_pending = false;
        }
        EnableGlobalIRQ(primask);

        if (loss_here)
        {
            g_rx_loss_events++;
            g_line_len = 0U;
            g_line_discard = true;
        }

        if (g_rx_tail == g_rx_head)
        {
            break;
        }

        uint8_t c = g_rx_ring[g_rx_tail & RX_RING_MASK];
        g_rx_tail++;
        Console_ProcessByte(c);
    }
}

static void Telemetry_Task(void)
{
    if (!g_stream_on || ((uint32_t)(g_ms - g_tel_last_ms) < 1000U))
    {
        return;
    }
    g_tel_last_ms += 1000U; // sin deriva
    PRINTF("TEL MODE=%s FREQ=%u DUTY=%u UPTIME=%u\r\n", State_Name(current_state), (unsigned)g_freq_hz,
           (unsigned)pwm_final, (unsigned)g_uptime_s);
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief ISR de la LPUART: solo mueve bytes al buffer circular y registra errores.
 */
void LPUART0_ISRQHANDLER_RX(void)
{
    LPUART_Type *base = (LPUART_Type *)BOARD_DEBUG_UART_BASEADDR;
    uint32_t flags = LPUART_GetStatusFlags(base);

    // Errores de hardware (overrun, ruido, framing, paridad): limpiar y registrar la perdida
    uint32_t err = flags & ((uint32_t)kLPUART_RxOverrunFlag | (uint32_t)kLPUART_NoiseErrorFlag |
                            (uint32_t)kLPUART_FramingErrorFlag | (uint32_t)kLPUART_ParityErrorFlag);
    if (err != 0U)
    {
        LPUART_ClearStatusFlags(base, err);
        Rx_MarkLoss();
    }

    // Leer TODO lo que haya en el FIFO, no solo un byte
    while (LPUART_GetRxFifoCount(base) > 0U)
    {
        Rx_Push(LPUART_ReadByte(base));
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

/* ISR del pulsador: ambos flancos. Solo reconoce la causa y registra el instante. */
void SW2_IRQHANDLER(void)
{
    // 1. Obtiene las banderas de interrupción del puerto del botón
    uint32_t int_flags = GPIO_GpioGetInterruptFlags(BOARD_INITPINS_SW_2_GPIO);
    uint32_t pin_mask = (1UL << BOARD_INITPINS_SW_2_GPIO_PIN);

    if (int_flags != 0U)
    {
        // 2. Limpia las banderas inmediatamente (propias y ajenas, por si el vector se comparte con otros pines)
        GPIO_GpioClearInterruptFlags(BOARD_INITPINS_SW_2_GPIO, int_flags);

        // 3. Si fue nuestro pin: registra el instante y notifica al bucle principal
        if ((int_flags & pin_mask) != 0U)
        {
            g_button_edge_ms = g_ms;
            g_button_edge_pending = true;
        }
    }
    SDK_ISR_EXIT_BARRIER; // Asegura el cierre correcto de la ISR en arquitecturas Cortex-M
}

/* Accion X (pulsacion corta): alterna MANUAL <-> AUTO. En PAUSA se ignora. */
static void Button_ShortPress(void)
{
    g_btn_short_cnt++;
    if (current_state == STATE_PAUSE)
    {
        g_btn_short_ignored++;
        PRINTF("Acción X: Pulsación Corta ignorada en PAUSA\r\n");
        return;
    }
    if (current_state == STATE_MANUAL)
    {
        Mode_EnterAuto();
    }
    else
    {
        Mode_EnterManual();
    }
    PRINTF("Acción X: Pulsación Corta detectada (< 1500ms) -> %s\r\n", State_Name(current_state));
}

/* Accion Y (pulsacion larga >= 1500 ms): entra/sale de PAUSA. */
static void Button_LongPress(void)
{
    g_btn_long_cnt++;
    if (current_state == STATE_PAUSE)
    {
        (void)Pause_Exit();
    }
    else
    {
        Pause_Enter();
    }
    PRINTF("Acción Y: Pulsación Larga detectada (>= 1500ms) -> %s\r\n", State_Name(current_state));
}

/* Toma el flanco pendiente (si hay) y su instante. */
static bool Button_TakeEdge(uint32_t *edge_ms)
{
    if (!g_button_edge_pending)
    {
        return false;
    }
    g_button_edge_pending = false;
    *edge_ms = g_button_edge_ms;
    return true;
}

void BUTTON_STATE_MACHINE(void)
{
    uint32_t now = g_ms;
    uint32_t edge_ms = 0;
    bool edge = Button_TakeEdge(&edge_ms);

    // --- 1. GESTIÓN DEL BOTÓN Y ANTIRREBOTE (30 ms de nivel estable, sin retardos activos) ---
    switch (btn_state)
    {
    case BTN_IDLE:
        if (edge)
        {
            btn_timer = edge_ms; // Inicia la ventana de antirrebote
            btn_state = BTN_DEBOUNCING_PRESS;
        }
        break;

    case BTN_DEBOUNCING_PRESS:
        if (edge)
        {
            btn_timer = edge_ms; // Otro flanco (rebote): reinicia la ventana
        }
        else if ((uint32_t)(now - btn_timer) >= BTN_DEBOUNCE_MS)
        {
            // Lee el estado físico real del pin (lógica invertida Pull-Up: 0 = presionado)
            uint32_t pin_val = GPIO_PinRead(BOARD_INITPINS_SW_2_GPIO, BOARD_INITPINS_SW_2_GPIO_PIN);

            if (pin_val == 0U) // Presión confirmada tras 30 ms estables
            {
                btn_press_ms = btn_timer; // la duración se mide desde el flanco de esta pulsación validada
                long_press_dispatched = false;
                btn_state = BTN_PRESSED;
            }
            else
            {
                btn_state = BTN_IDLE; // Fue ruido, no hubo pulsación estable
            }
        }
        break;

    case BTN_PRESSED:
    {
        // Pulsación larga: una sola vez al alcanzar 1500 ms (sin repeticiones al mantener)
        uint32_t ref = edge ? edge_ms : now;
        if (!long_press_dispatched && ((uint32_t)(ref - btn_press_ms) >= BTN_LONG_MS))
        {
            long_press_dispatched = true;
            Button_LongPress(); // ---> INSTRUCCIÓN Y <---
        }

        // Candidato a liberación: flanco por interrupción (o lectura del pin como respaldo)
        if (edge)
        {
            btn_timer = edge_ms;
            btn_state = BTN_DEBOUNCING_RELEASE;
        }
        else if (GPIO_PinRead(BOARD_INITPINS_SW_2_GPIO, BOARD_INITPINS_SW_2_GPIO_PIN) != 0U)
        {
            btn_timer = now;
            btn_state = BTN_DEBOUNCING_RELEASE;
        }
    }
    break;

    case BTN_DEBOUNCING_RELEASE:
        if (edge)
        {
            btn_timer = edge_ms; // Rebote al soltar: reinicia la ventana
        }
        else if ((uint32_t)(now - btn_timer) >= BTN_DEBOUNCE_MS)
        {
            uint32_t pin_val = GPIO_PinRead(BOARD_INITPINS_SW_2_GPIO, BOARD_INITPINS_SW_2_GPIO_PIN);

            if (pin_val != 0U) // Liberación confirmada
            {
                if (!long_press_dispatched)
                {
                    // Duración sobre el estado ya validado: flanco de liberación - flanco de pulsación
                    if ((uint32_t)(btn_timer - btn_press_ms) >= BTN_LONG_MS)
                    {
                        Button_LongPress(); // superó 1500 ms (con ruido al soltar): cuenta como larga
                    }
                    else
                    {
                        Button_ShortPress(); // ---> INSTRUCCIÓN X <---
                    }
                }
                // Si la larga ya se disparó, al soltar NO se genera corta
                btn_state = BTN_IDLE;
            }
            else
            {
                btn_state = BTN_PRESSED; // Seguía presionado: continúa la misma pulsación
            }
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

    /* Pulsador: interrupción en AMBOS flancos (si ya lo configuraste así en el Pins Tool, puedes quitar esta línea) */
    GPIO_SetPinInterruptConfig(BOARD_INITPINS_SW_2_GPIO, BOARD_INITPINS_SW_2_GPIO_PIN, kGPIO_InterruptEitherEdge);

    /* Estado inicial: MANUAL, 1 kHz, duty 25 %, LED fijo */
    pwm_final = g_manual_duty;
    Pwm_Configure(PWM_FREQ_INITIAL, pwm_final);
    LED_Restart();

    PRINTF("Iniciamos el programa\r\n");
    PRINTF("PWM_init: %d%%\r\n", pwm_final);
    PRINTF("PWM: clk=%u Hz PR=%u timer=%u Hz FREQ=%u Hz MR_PERIODO=%u\r\n", (unsigned)g_pwm_clk_hz,
           (unsigned)g_pwm_pr, (unsigned)g_pwm_timer_hz, (unsigned)g_freq_hz, (unsigned)g_pwm_period_reg);

    while (1)
    {
        // 1. Botón (antirrebote, flancos, corta/larga)
        BUTTON_STATE_MACHINE();

        // 2. Consola UART: buffer circular -> líneas -> comandos (CR, LF y CRLF)
        Console_Poll();

        // 3. Rampa AUTO, LED de estado y telemetría (STREAM ON), todo con g_ms sin bloquear
        Auto_Task();
        LED_Task();
        Telemetry_Task();

        // 4. Tarea periódica de 1 segundo
        if ((uint32_t)(g_ms - last_tick) >= 1000U)
        {
            last_tick += 1000U;
            g_uptime_s++;
        }
    }
}