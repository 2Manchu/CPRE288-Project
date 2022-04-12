/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "servo.h"
#include "Timer.h"
#include "button.h"
#include "lcd.h"

uint32_t RIGHT_CAL_VALUE = 0;
uint32_t LEFT_CAL_VALUE = 0;
int calibrated = 0;

void servo_init (void) {
    //Enable clocks to GPIO and Timer
    SYSCTL_RCGCGPIO_R |= 0b00010;
    SYSCTL_RCGCTIMER_R |= 0b000010;
    while ((SYSCTL_PRGPIO_R & 0b00010) == 0 && (SYSCTL_PRTIMER_R & 0b000010) == 0) {};

    //Configure Port B5, digital on, then set it to alternate select and choose timer 1B mode
    GPIO_PORTB_DEN_R |= 0b100000;
    GPIO_PORTB_DIR_R |= 0b100000;
    GPIO_PORTB_AFSEL_R |= 0b100000;
    GPIO_PORTB_PCTL_R |= 0x700000;

    //Configure timer 1B (not enabled yet): 16 bit mode, pos edge trigger, periodic mode, PWM enable
    // set reset value to FFFF, prescale reset to FF
    TIMER1_CTL_R &= 0xFEFF;
    TIMER1_CFG_R |= 0x04;
    TIMER1_TBMR_R |= 0b1010;

    //Set base timer and prescaler to have a total signal period of 20ms (4E200).
    //Pre-configure match register to position servo at 90 degrees by default
    TIMER1_TBILR_R = 0xE200;
    TIMER1_TBPR_R |= 0x04;
    //get degrees for 16 bit match register by looking at returned lower 16 bits, then the prescaler match by looking at 24:16 of returned number
    TIMER1_TBMATCHR_R = (24000 & 0xFFFF);
    TIMER1_TBPMR_R = ((24000 >> 16) & 0xFF);

    //enable the timer
    TIMER1_CTL_R |= 0x4100;
}

void servo_move(uint16_t degrees) {
    uint32_t matchVal = calc_duty_cycle(degrees);

    //disable the timer
    TIMER1_CTL_R &= 0xFEFF;

    TIMER1_TBMATCHR_R = matchVal & 0xFFFF;
    TIMER1_TBPMR_R = (matchVal >> 16) & 0xFF;

    TIMER1_CTL_R |= 0x4100;
}

//TODO: basically just have buttons to go left and right that add/subtract a set amount of duty cycle per iteration, then confirm the left and right values with other buttons
_Noreturn int servo_calibrate() {
    uint32_t currentPulseWidth = 24000;
    TIMER1_CTL_R &= 0xFEFF;
    TIMER1_TBMATCHR_R = currentPulseWidth & 0xFFFF;
    TIMER1_TBPMR_R = currentPulseWidth >> 16;
    TIMER1_CTL_R |= 0x4100;

    while (1) {
        lcd_printf("Pulse width: %d\nL val: %d\nR val: %d", currentPulseWidth, LEFT_CAL_VALUE, RIGHT_CAL_VALUE);
        int button = button_getButton();
        timer_waitMillis(50);
        if (button == 1) {
            currentPulseWidth += 100;
            TIMER1_CTL_R &= 0xFEFF;
            TIMER1_TBMATCHR_R = currentPulseWidth & 0xFFFF;
            TIMER1_TBPMR_R = (currentPulseWidth >> 16) & 0xFF;
            TIMER1_CTL_R |= 0x4100;
        }
        else if (button == 2) {
            currentPulseWidth -= 100;
            TIMER1_CTL_R &= 0xFEFF;
            TIMER1_TBMATCHR_R = currentPulseWidth & 0xFFFF;
            TIMER1_TBPMR_R = (currentPulseWidth >> 16) & 0xFF;
            TIMER1_CTL_R |= 0x4100;
        }
        else if (button == 3) {
            LEFT_CAL_VALUE = currentPulseWidth;
        }
        else if (button == 4) {
            RIGHT_CAL_VALUE = currentPulseWidth;
        }
    }
}

void set_left(uint32_t clocks) {
    LEFT_CAL_VALUE = clocks;
}

void set_right(uint32_t clocks) {
    RIGHT_CAL_VALUE = clocks;
}


//TODO: Have left and rightmost duty cycle (# clocks) values that you find by interacting, then calculate #clocks per degree and multiply the desired degrees times that and add base value for 0deg
uint32_t calc_duty_cycle(uint16_t degrees) {

    uint32_t clocksPerDegree = (LEFT_CAL_VALUE - RIGHT_CAL_VALUE) / 180.0;
    //We add right value because if we have 0 degrees that's the motor facing right 90 degrees of center
    uint32_t dutyCycleClocks = (clocksPerDegree * degrees) + RIGHT_CAL_VALUE;

    return dutyCycleClocks;
}
