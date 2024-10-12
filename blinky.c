#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h" // Incluindo cabeçalho para interrupções
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h" // Incluindo cabeçalho para interrupções
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

//
//--------------------------------------- Manipulador de interrupção da UART ---------------------------------------
//
//void UARTIntHandler(void)
//{
//}

/*
Função: UARTInit
Descrição:
Esta função inicializa a UART0 do microcontrolador.
Ela habilita o clock para a UART, espera até que
o módulo esteja pronto, e então configura os parâmetros
da UART, como a taxa de transmissão, bits de dados,
paridade e bits de parada.
*/
void UARTInit (void)
    {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // habilitando

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) //espera o modulo UART0 estar pronto
        {
        }
//
// ---------------------------------- Ativa o módulo UART0 e GPIOA ----------------------------------------------
//

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);// PA0 como RX
    GPIOPinConfigure(GPIO_PA1_U0TX);// PA1 como TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
// ----------------------- Configurar a UART: 115200  bps, 8 bits, sem paridade, 1 stop bit --------------------------------
//
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200 ,
                       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    }

/*
   Função:
   Descrição:

*/


uint32_t receiveINT32 (void)
    {
    uint_fast16_t  i;
    uint32_t receiveValInt = 0; // valor atualizado para receber o inteiro da UART
    while(!UARTCharsAvail(UART0_BASE)) {}; //verifica a presença de caracteres na FIFO de recebimento da UART
    for (i = 0; i < 4 ;i++) // 4 bytes de 8 bits
        {
        receiveValInt |= (UARTCharGet(UART0_BASE) << (i*8)); // recebe um dado desloca o bit LSB e muda os novos bits = 1
        }
    return receiveValInt;
    }

/*
   Função:
   Descrição:

*/
void SendInt32 (uint32_t ValInt)
    {
    uint_fast16_t  i1;
    for (i1 =0; i1 < 4 ;i1++) // 4 bytes de 8 bits
            {
            UARTCharPut(UART0_BASE, (ValInt >> (i1*8))); // envia um dado desloca o bit LSB
            }
    }


int main(void) {
//
//--------------------------------------- clk 120 MHz ------------------------------------------------------------------------------
//
SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);

// -------------------------------------- INICIALIZA A UART -----------------------------------------------------
UARTInit();

// ------------------------------------- loop infinito ------------------------------------------------------------
while(1)
{
    uint32_t numberToSend = 120; // Exemplo de número a ser enviado

   // Envia um número inteiro de 32 bits
    SendInt32(numberToSend);


   // Recebe um número inteiro de 32 bits
   uint32_t receivedNumber = receiveINT32();
}
}
