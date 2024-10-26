#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"  // Necessário para configurar os pinos UART
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"


#define SIZE_BUFFER 1000
//uint8_t buffer[SIZE_BUFFER];  // Buffer para armazenar os 4 bytes recebidos
int buffer[SIZE_BUFFER];
uint32_t ui32SysClkFreq;
// Função para inicializar a UART
void UARTInit(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8 |
UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));


    UARTEnable(UART0_BASE);
}


// Função para enviar 4 bytes via UART
void sendINT32(int* buffer) {
    uint_fast16_t i;
    char* ptr = (char*)buffer;
    for ( i = 0; i < SIZE_BUFFER*sizeof(int) ; i++) {
        UARTCharPut(UART0_BASE, ptr[i]);  // Envia cada byte
    }
}

int main(void) {
    // Configura o sistema para rodar a 120 MHz
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    // Inicializa a UART
    UARTInit();
    uint16_t i;
    for(i =0; i < SIZE_BUFFER; i++)
    {
        buffer[i] = i;
    }
    while (1) {
        // Envia de volta os 4 bytes recebidos
        sendINT32(buffer);
    }
}
