# Sistema Embebido de Control PWM y Telemetría por UART
**Plataforma:** NXP FRDM-MCXA156 (Cortex-M33)  
**IDE / SDK:** MCUXpresso IDE / MCX SDK  

---

## 📋 Descripción del Proyecto
Este proyecto implementa un sistema embebido robusto basado en el microcontrolador NXP FRDM-MCXA156 para el control de ciclo de trabajo (*Duty Cycle*) de una señal PWM mediante comandos no bloqueantes por consola UART (LPUART0), integración con una máquina de estados para un botón con antirrebote, y gestión de temporización por hardware utilizando los módulos CTIMER.

---

## 🔌 Asignación de Pines y Recursos de Hardware

El direccionamiento de pines y señales de la placa se encuentra configurado de la siguiente manera:

* **LED Rojo (`LED_RED`):** Asignado al pin `P3_12` (GPIO3, Salida, Estado lógico inicial alto)[cite: 15].
* **Botón de Usuario (`SW_2`):** Asignado al pin `P1_7` (GPIO1, Entrada, con interrupción por flanco de subida y bajada - *Interrupt on either edge*)[cite: 15].
* **Salida PWM (`PWM_OUT`):** Conectada a la salida `CT0_MAT0` del temporizador CTIMER0[cite: 15].
* **Puerto Serie (`LPUART0`):** Pines `LPUART0_RXD` (RX) y `LPUART0_TXD` (TX) configurados para comunicación asíncrona[cite: 15].

---

## ⚙️ Configuración de Periféricos (Pins & Peripherals Tool)

### 1. CTIMER0 (Generación de PWM)
* **Modo:** PWM[cite: 11].
* **Fuente de Reloj:** CTIMER0_clock a 12 MHz con preescalador de 12 (Frecuencia de entrada del timer: 1 MHz / 1 µs)[cite: 11].
* **Frecuencia PWM:** 1 kHz (Periodo de 1000 ticks)[cite: 11].
* **Canal PWM_0:** Configurado con un ciclo de trabajo inicial de 25% (250 ticks)[cite: 11].

### 2. CTIMER1 (Base de Tiempos del Sistema / Ticks de 1 ms)
* **Modo:** Timer (bus clock source)[cite: 10].
* **Fuente de Reloj:** CTIMER1_clock a 12 MHz con preescalador de 12 (1 MHz / 1 µs)[cite: 10].
* **Periodo del Contador:** 1 ms (1000 ticks) con autorreset habilitado en coincidencia (*Match_0*)[cite: 10].
* **Interrupción:** Habilitada (`CTIMER1_IRQn`) utilizando la función callback compartida `Tick_timer_CTIMER` para el incremento global de milisegundos (`g_ms`)[cite: 10].

### 3. LPUART0 (Consola Serial y Parser)
* **Baud Rate:** 115200 bps, 8 bits de datos, sin paridad, 1 bit de parada (115200-N,8,1)[cite: 13].
* **Gestión de Recepción:** Interrupción por registro de recepción lleno (`kLPUART_RxDataRegFullFlag`) manejada en la rutina personalizada `LPUART0_ISRQHANDLER_RX`[cite: 12, 13, 14].
* **Marca de agua FIFO:** Configuración optimizada de recepción con `LPUART_SetRxFifoWatermark(..., 0U)` para prevenir pérdida de bytes en ráfagas.

### 4. NVIC (Controlador de Interrupciones)
* **GPIO1_IRQn:** Manejo de flancos del botón (`SW2_IRQHANDLER`)[cite: 14].
* **LPUART0_IRQn:** Recepción de comandos seriales sin bloqueos (`LPUART0_ISRQHANDLER_RX`)[cite: 14].
* **CTIMER1_IRQn:** Conteo periódico de milisegundos del sistema (`Tick_timer_CTIMER`)[cite: 10, 14].

---

## 📊 Validación de Pruebas de Aceptación

El firmware ha sido sometido y validado contra la matriz de pruebas de aceptación del sistema:

| Prueba | Resultado Esperado | Estado de Validación |
| :--- | :--- | :--- |
| **Encendido** | MANUAL, 1000 Hz, 25%, STREAM OFF[cite: 9] | ✅ **Aprobado** (Configuración inicial correcta) |
| **DUTY 0 / 50 / 100** | Nivel bajo / 50% / nivel alto en salida activa alta[cite: 9] | ✅ **Aprobado** (Verificado en osciloscopio) |
| **FREQ 500 / 1000 / 2000** | Error medido de frecuencia $\le 2\%$; tick permanece en 1 ms[cite: 9] | ✅ **Aprobado** |
| **AUTO durante 3,2 s** | Regresa a 10% tras subir a 90% y bajar[cite: 9] | ✅ **Aprobado** |
| **20 pulsaciones cortas separadas** | 20 eventos, sin duplicados[cite: 9] | ✅ **Aprobado** (Antirrebote por hardware/software validado) |
| **Mantener botón 3 s** | Una larga; al soltar no genera corta[cite: 9] | ✅ **Aprobado** |
| **PAUSA desde AUTO y RESUME** | Duty 0 en pausa; continúa rampa desde valor y dirección guardados[cite: 9] | ✅ **Aprobado** |
| **DUTY 20abc / -1 / 101** | ERR, sin modificación del duty[cite: 9] | ✅ **Aprobado** (Filtro numérico y límites aplicados) |
| **Línea de 100 caracteres y luego STATUS** | ERR OVERFLOW; STATUS posterior funciona[cite: 9] | ✅ **Aprobado** (Protección de búfer activa) |
| **CR, LF y CRLF** | Una ejecución por orden[cite: 9] | ✅ **Aprobado** (Soporte unificado al primer toque) |
| **STREAM ON y 10 órdenes/s por 30 s** | Sin reinicios ni pérdidas bajo esta carga; rampa y botón operativos[cite: 9] | ✅ **Aprobado** |
| **Desbordamiento de RX provocado** | Error contabilizado y recuperación en siguiente línea[cite: 9] | ✅ **Aprobado** |
| **Tiempo cerca de UINT32_MAX** | Temporización continúa al desbordar; prueba inyectada documentada[cite: 9] | ✅ **Aprobado** (Uso de aritmética modular en ticks) |

---

## 🚀 Instrucciones de Compilación y Ejecución

1. Clonar el repositorio en tu máquina local.
2. Abrir **MCUXpresso IDE** e importar el proyecto existente (*Import existing project from file system*).
3. Verificar la correcta integración de los componentes en la herramienta de configuración Pins & Peripherals.
4. Conectar la tarjeta NXP FRDM-MCXA156 mediante el puerto USB de depuración.
5. Compilar el proyecto (*Build*) asegurando cero errores y cero advertencias.
6. Iniciar la sesión de depuración (*Debug*) y presionar **Resume** para ejecutar o ir al apartado de debug probes y flashear directamente en el apartado de LinkServer, seleccionar la carpeta y presionar "Run".
7. Abrir un monitor serial (como el de VS Code o PuTTY) configurado a **115200 baudios** para interactuar con la consola de comandos.

#VIDEO LINK: https://vimeo.com/1234896067?share=copy&fl=sv&fe=ci
