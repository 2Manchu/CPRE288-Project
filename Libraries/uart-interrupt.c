/*
*
*   uart-interrupt.c
*
*
*
*   @author Anthony Manschula, Karthik Kasarabada
*   @date 3-9-22
*/


#include <inc/tm4c123gh6pm.h>
#include "tm4c123gh6pm.h"
#include "stdbool.h"
#include <stdint.h>
#include "uart-interrupt.h"
#include "driverlib/interrupt.h"

// These variables are declared as examples for your use in the interrupt handler.
volatile int manualMode = 0;
volatile int manualKey = 109;
volatile char stop_byte = 111;  //letter 'o' makes robot stop
volatile char go_byte = 112;    //letter 'p' makes robot go brr
volatile int goCmd = 0;         //command that tells starts execution
volatile int goForward = 119;   //letter 'w' goes forward
volatile int goBackward = 115;  //letter 's' goes backward
volatile int turnLeft = 97;     //letter 'a' turns left
volatile int turnRight = 100;   //letter 'd' turns right
volatile int movementCode = -1;

void uart_interrupt_init(void){
  //enable clock to GPIO port B
  SYSCTL_RCGCGPIO_R |= 0b000010;

  //enable clock to UART1
  SYSCTL_RCGCUART_R |= 0b00000010;

  //wait for GPIOB and UART1 peripherals to be ready
  while ((SYSCTL_PRGPIO_R & 0b000010) == 0) {};
  while ((SYSCTL_PRUART_R & 0b00000010) == 0) {};

  //enable alternate functions on port B pins
  GPIO_PORTB_AFSEL_R |= 0b00000011;

  //enable digital functionality on port B pins
  GPIO_PORTB_DEN_R |= 0b00000011;

  //enable UART1 Rx and Tx on port B pins
  //NOTE: In hex because each entry for the port control register accepts a 4 bit number (found on pg650 of datasheet)
  GPIO_PORTB_PCTL_R |= 0x00000011;

  //calculate baud rate
  uint16_t iBRD = 8; //use equations
  uint16_t fBRD = 44; //use equations

  //turn off UART1 while setting it up
  UART1_CTL_R &= 0b0;

  //set baud rate
  //note: to take effect, there must be a write to LCRH after these assignments
  UART1_IBRD_R = iBRD;
  UART1_FBRD_R = fBRD;

  //set frame, 8 data bits, 1 stop bit, no parity, no FIFO
  //note: this write to LCRH must be after the BRD assignments
  UART1_LCRH_R |= 0b01100000;

  //use system clock as source
  //note from the datasheet UARTCCC register description:
  //field is 0 (system clock) by default on reset
  //Good to be explicit in your code
  UART1_CC_R = 0x0;

  //////Enable interrupts

  //first clear RX interrupt flag (clear by writing 1 to ICR)
  UART1_ICR_R |= 0b00010000;

  //enable RX raw interrupts in interrupt mask register
  UART1_IM_R |= 0x0010;

  //NVIC setup: set priority of UART1 interrupt to 1 in bits 21-23
  NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

  //NVIC setup: enable interrupt for UART1, IRQ #6, set bit 6
  NVIC_EN0_R |= 0x00000040;

  //tell CPU to use ISR handler for UART1 (see interrupt.h file)
  //from system header file: #define INT_UART1 22
  IntRegister(INT_UART1, UART1_Handler);

  //globally allow CPU to service interrupts (see interrupt.h file)
  IntMasterEnable();

  //re-enable UART1 and also enable RX, TX (three bits)
  //note from the datasheet UARTCTL register description:
  //RX and TX are enabled by default on reset
  //Good to be explicit in your code
  //Be careful to not clear RX and TX enable bits
  //(either preserve if already set or set them)
  UART1_CTL_R |= 0x0301;

}

void uart_sendChar(char data){
    while((UART1_FR_R & 0x20) != 0);
    UART1_DR_R = data;
}

char uart_receive_blocking(void){
    char recData;

    //Wait while there is nothing to receive
    while((UART1_FR_R & 0x10) != 0);

    //Error Check
    if (UART1_DR_R & 0xF00) {
        GPIO_PORTB_DATA_R = 0xF;
    }
    else {
        recData = (char)UART1_DR_R & 0xFF;
    }
    return recData;
}

char uart_receive(void){
    char recData;

    //Error Check
    if (UART1_DR_R & 0xF00) {
        GPIO_PORTB_DATA_R = 0xF;
    }
    else {
        recData = (char)UART1_DR_R & 0xFF;
    }
    return recData;
}

//TODO: UART SENDSTRING
void uart_sendStr(const char *data){

}

// Interrupt handler for receive interrupts
void UART1_Handler(void)
{
    char byte_received;
    //check if handler called due to RX event
    if (UART1_MIS_R & 0x10)
    {
        //byte was received in the UART data register
        //clear the RX trigger flag (clear by writing 1 to ICR)
        UART1_ICR_R |= 0b00010000;

        //read the byte received from UART1_DR_R and echo it back to PuTTY
        //ignore the error bits in UART1_DR_R
        byte_received = uart_receive();
        uart_sendChar(byte_received);

        //if byte received is a carriage return
        if (byte_received == '\r')
        {
            //send a newline character back to PuTTY
            uart_sendChar('\n');
        }
        else
        {
            //AS NEEDED
            //code to handle any other special characters
            //code to update global shared variables
            //DO NOT PUT TIME-CONSUMING CODE IN AN ISR

            if (byte_received == go_byte)
            {
              goCmd = 1;
            }
            if (byte_received == manualKey) {
                manualMode = !manualMode;
            }
            else if (byte_received == stop_byte) {
                goCmd = 0;
            }
            else if (byte_received == goForward) {
                movementCode = 1;
            }
            else if (byte_received == goBackward) {
                movementCode = 2;
            }
            else if (byte_received == turnLeft) {
                movementCode = 3;
            }
            else if (byte_received == turnRight) {
                movementCode = 4;
            }

        }
    }
}
