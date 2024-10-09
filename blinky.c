#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#define A 127          // Amplitude (max: 255)
#define F 1            // Frequência em Hz
#define SAMPLING_RATE 100 // Taxa de amostragem em Hz

uint32_t g_ui32SysClock;

int main(void) {
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    uint32_t t = 0; // Tempo em milissegundos
    while (1) {
        // Calcula o valor da senoide
        float value = A * sin(2 * M_PI * F * (t / 1000.0)); // t em segundos
        uint8_t sendValue = (uint8_t)(value + 128); // Centraliza em 0-255

        // Envia o valor pela UART
        UARTCharPut(UART0_BASE, sendValue);

        // Espera pelo intervalo de amostragem
        SysCtlDelay(g_ui32SysClock / SAMPLING_RATE);
        t += (1000 / SAMPLING_RATE); // Incrementa o tempo 1000hz,
    }
}
