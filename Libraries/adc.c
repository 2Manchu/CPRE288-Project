//
// Created by Tony Manschula on 3/23/22.
//

#include "adc.h"
#include "lcd.h"
#include "timer.h"
#include "math.h"

void adc_init() {
    //Set the GPIO port B clock
    SYSCTL_RCGCGPIO_R |= 0b000010;
    while((SYSCTL_PRGPIO_R & 0b000010) != 0x02) {};

    //Select Ain for pin 4 of port b, disable digital, set it as input, and disable analog isolation
    GPIO_PORTB_AFSEL_R |= 0x10;
    GPIO_PORTB_DEN_R &= ~0x10;
    GPIO_PORTB_DIR_R &= ~0x10;
    GPIO_PORTB_AMSEL_R |= 0x10;

    //Enable ADC0 clock
    SYSCTL_RCGCADC_R |= 0b01;
    while ((SYSCTL_PRADC_R & 0b01) != 0b01) {};

    //Set the port control sample rate to 125k/sec
    ADC0_PC_R &= ~0xF;
    ADC0_PC_R |= 0x1;
    //Set sequencer 3 to highest priority (1 bit depth register)
    ADC0_SSPRI_R = 0x0123;

    //Disable sequencer 3 and set it to software trigger mode (0)
    //Q: What is software trigger mode??
    ADC0_ACTSS_R &= 0b0111;
    ADC0_EMUX_R &= ~0xF000;
    //Clear the selected analog input for sequencer 3 and then select channel 10 for sequencer 3
    ADC0_SSMUX3_R &= 0b0000;
    ADC0_SSMUX3_R += 10;
    //Configure some parameters for the sample sequencer, idk look at pg 876 of datasheet for more info
    ADC0_SSCTL3_R = 0x6;
    ADC0_IM_R &= 0b0111;
    ADC0_SAC_R = 6;

    //Re-enable the sequencer 3
    ADC0_ACTSS_R |= 0b1000;
}

uint16_t adc_read() {
    uint32_t result = -1;

    ADC0_PSSI_R = 0x0008;
    while ((ADC0_RIS_R & 0x08) == 0) {}
    result = ADC0_SSFIFO3_R & 0xFFF;
    ADC0_ISC_R = 0x0008;
    return result;

}

void adc_print(uint32_t irVal, double dist) {
    lcd_printf("IR Value: %u\nDistance: %lf", irVal, dist);
}

double adc_getDistance(uint32_t irVal) {
    return 30255 * pow(irVal, -1.01);
}