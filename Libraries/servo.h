/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

/**
 * Initialize servo motor. Uses PB5 and Timer 1B
 */
void servo_init (void);

void servo_move(uint16_t degrees);

_Noreturn int servo_calibrate(void);

void set_right(uint32_t clocks);
void set_left(uint32_t clocks);

/**
 * Calculate the time in clock cycles, given an amount of degrees, that
 * the high portion of the signal should last
 */
uint32_t calc_duty_cycle(uint16_t degrees);
