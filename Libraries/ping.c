/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping.h"
#include "Timer.h"
#include "lcd.h"

volatile unsigned long START_TIME = 0;
volatile unsigned long END_TIME = 0;
volatile enum {LOW, HIGH, DONE} STATE = LOW;

void ping_init (void) {
    //Enable clocks to GPIO and Timer
    SYSCTL_RCGCGPIO_R |= 0b00010;
    SYSCTL_RCGCTIMER_R |= 0b001000;
    while ((SYSCTL_PRGPIO_R & 0b00010) == 0 && (SYSCTL_PRTIMER_R & 0b001000) == 0) {};

    //Configure Port B3, digital on, then set it to alternate select and choose timer 3B mode
    GPIO_PORTB_DEN_R |= 0b1000;
    GPIO_PORTB_AFSEL_R |= 0b1000;
    GPIO_PORTB_PCTL_R |= 0x7000;

    //Configure timer 3B (not enabled yet): both edge trigger, 16 bit mode, capture + edge-time mode + down count,
    // set reset value to FFFF, prescale reset to FF
    TIMER3_CTL_R |= 0x0C00;
    TIMER3_CFG_R |= 0x4;
    TIMER3_TBMR_R |= 0x007;
    TIMER3_TBILR_R |= 0xFFFF;
    TIMER3_TBPR_R |= 0xFF;

    //Configure timer interrupts for event capture mode, and set the priority of the timer interrupt, and enable the interrupt in the NVIC
    TIMER3_IMR_R |= 0x400;
    TIMER3_ICR_R |= 0x400;
    NVIC_PRI9_R |= 0x20;
    NVIC_EN1_R |= 0b10000;

    // Configure and enable the timer (this is 0 D 0 0 where the second on the left is a D as in delta)
    TIMER3_CTL_R |= 0x0D00;

  //Set the location of our interrupt handler for timer 3b
    IntRegister(INT_TIMER3B, TIMER3B_Handler);
    IntMasterEnable();
}

void ping_trigger (void) {
    STATE = LOW;
    //Clear the data register for portB3
    GPIO_PORTB_DATA_R &= 0b11110111;
    // Disable timer3B and disable timer3B event capture mode interrupt
    TIMER3_CTL_R &= 0b0;
    TIMER3_IMR_R &= 0xBFF;
    // Disable alternate function (disconnect timer from port pin) and set as output
    GPIO_PORTB_AFSEL_R &= 0b0111;
    GPIO_PORTB_DIR_R |= 0b1000;

    //Set high for 5 microseconds to trigger start
    GPIO_PORTB_DATA_R |= 0b1000;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R &= 0b11110111;

    //Set B3 back to input
    GPIO_PORTB_DIR_R &= 0b0111;

    // Clear an interrupt that may have been erroneously triggered
    TIMER3_ICR_R |= 0x400;
    // Re-enable alternate function, timer interrupt, and timer
    GPIO_PORTB_AFSEL_R |= 0b1000;
    TIMER3_IMR_R |= 0x400;
    TIMER3_CTL_R |= 0x0D00;
}

void TIMER3B_Handler(void) {
    if (TIMER3_RIS_R & 0x400) {
        //Clear Interrupt status
        TIMER3_ICR_R |= 0x400;

        if (STATE == LOW || STATE == DONE) {
            //Can't you just take the value of the GPTMTBR register since it contains the value of the
            START_TIME = (TIMER3_TBR_R & 0xFFFFFF);
            STATE = HIGH;
        } else if (STATE == HIGH) {
            END_TIME = (TIMER3_TBR_R & 0xFFFFFF);
            STATE = DONE;
        }
    }

}

float ping_getDistance (void) {
    unsigned long clockPulses = 0;
    unsigned long offset = 0xFF<<16;
    float speedSound = 343.0;
    float timePerClock = .0000000625;
    float reflectionTime = 0;

    ping_trigger();
    timer_waitMillis(50);
    if (START_TIME < END_TIME) {
        clockPulses = (START_TIME - END_TIME) + offset;
        reflectionTime = (clockPulses * timePerClock) / 2.0;
        lcd_printf("%f meters + overflow", reflectionTime * speedSound);
        return reflectionTime * speedSound;
    }
    else {
        clockPulses = START_TIME - END_TIME;
        reflectionTime = (clockPulses * timePerClock) / 2.0;
        lcd_printf("%f meters", reflectionTime * speedSound);
        return reflectionTime * speedSound;
    }
}
