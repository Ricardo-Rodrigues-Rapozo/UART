#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
//
// -------------------------- Variaveis Globais ------------------------------------------------------
//

uint32_t ui32SysClkFreq;
volatile uint32_t flag = 0; // Declare como volatile para indicar que pode mudar em interrupções
char buffer[32]; // Buffer para armazenar até 32 caracteres
char* ptr = buffer; // ptr aponta para o início do buffer
int inteiro; // Declara uma variável inteira

//
// -------------------------- Uart interrupt ------------------------------------------------------
//
void UARTIntHandler(void)// entra aqui quando recebe ou envia dados rx ou tx
{
flag = 1;
uint32_t ui32Status;
ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
{
UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE)); // echo
}
}

int main(void)
{
//
// -------------------------- Definindo o CLK ------------------------------------------------------
//
ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000); // Variavel para armazenar a frequenci de clk do sistema
//
// -------------------------- Habilitando os perifericos ------------------------------------------------------
//
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
//
// -------------------------- Configurações do GPIO ------------------------------------------------------
//
GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);
//
// -------------------------- Configurações do GPIO para UART ------------------------------------------------------
//
GPIOPinConfigure(GPIO_PA0_U0RX);
GPIOPinConfigure(GPIO_PA1_U0TX);
GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
// -------------------------- Configuração da UART ------------------------------------------------------
//
UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
IntMasterEnable();
IntEnable(INT_UART0);
UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);


//
// --------------------------   Loop Infinito para Recepção de Dados ------------------------------------------------------
//
while (1)
{
    //Um loop infinito que verifica se há caracteres disponíveis para leitura na UART.
    //Se houver, o caractere é recebido e imediatamente enviado de volta pela UART,
    //funcionando como um eco.
//    if (flag == 1) {
//        ptr = buffer; // Reseta o ponteiro para o início do buffer
////        for (int i = 0; i < 31; i++)
////        {
////            *ptr = UARTCharGet(UART0_BASE); // Lê um caractere da UART
////            if (*ptr == '\0') { // Se a leitura for nula, sai do loop
////                break; // Interrompe se encontrar o final que é a leitura do final de envio de dados
////            }
////            ptr++; // Move para a próxima posição no buffer
////        }
//        *ptr = '\0'; // Termina o buffer com um caractere nulo
//        flag = 0; // Reseta a flag
//    }

}
}
