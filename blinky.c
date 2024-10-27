#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"  // Necessário para configurar os pinos UART
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
    //
    //------------------ PARAMETROS --------------------------------------------------
    //
#define SIZE_BUFFER 1000
uint8_t SYNC_BYTE = 1;  // Sinal de sincronização esperado
volatile uint8_t flag = 0;
uint32_t control1 = 0; // variavel para debug
uint32_t control2; // variavel para debug
int buffer[SIZE_BUFFER];
uint32_t ui32SysClkFreq;

    //
    //--------------------Interrupção para RX UART -----------------------------------
    //

void UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        if(UARTCharGetNonBlocking(UART0_BASE) == SYNC_BYTE)
        {
            flag = 1;
        }
    }
}


void UARTInit(void) {
    //
    //--------------------Enable the UART peripheral -----------------------------------
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    //--------------------Set the Rx/Tx pins as UART pins -----------------------------------
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //
    //-------------------- Configure the UART baud rate, data configuration -----------------------------------
    //
    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    //
    //-------------------- Configure the Interrupt -----------------------------------
    //
    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable( UART0_BASE , UART_INT_RX | UART_INT_RT);

    //
    //-------------------- Enable UART -----------------------------------
    //
    //UARTEnable(UART0_BASE);
}



void sendINT32(int* buffer) {// Função para enviar 4 bytes via UART
    uint_fast16_t i;
    char* ptr = (char*)buffer;
    for ( i = 0; i < SIZE_BUFFER*sizeof(int) ; i++)
    {
        UARTCharPut(UART0_BASE, ptr[i]);  // Envia cada byte
    }
}





int main(void) {
//
//------------------ Configura o sistema para rodar a 120 MHz --------------------------------------------------
//
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
//
//------------------ Inicializa a UART --------------------------------------------------
//
    UARTInit();

//
//-----------------------------------------------------------------------------------
//
    uint16_t i;
    for(i =0; i < SIZE_BUFFER; i++)
    {
        buffer[i] = i;
    }
    while (1)
    {
        if(flag == 1)
        {
            sendINT32(buffer);
            flag = 0;
        }
    }
}
