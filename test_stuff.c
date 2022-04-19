//
// Created by Tony Manschula on 4/13/22.
//

#include "Libraries/Timer.h"
#include "Libraries/lcd.h"
#include "Libraries/adc.h"
#include "Libraries/button.h"
#include "Libraries/ping.h"
#include "Libraries/servo.h"
#include "Libraries/uart-interrupt.h"
#include "Libraries/open_interface.h"


void main() {


    int i;
    timer_init();
    lcd_init();
    adc_init();
    button_init();
    ping_init();
    servo_init();
    uart_interrupt_init();

    /*
    //servo_calibrate();
    set_left(35600);
    set_right(7500);

    for (i = 0; i < 3; i++) {
        lcd_printf("Ping Distance: %f", ping_getDistance());
        timer_waitMillis(1000);
    }

    timer_waitMillis(1000);
    servo_move(0);
    timer_waitMillis(1000);
    servo_move(45);
    timer_waitMillis(1000);
    servo_move(90);
    timer_waitMillis(1000);
    servo_move(135);
    timer_waitMillis(1000);

    uart_sendChar('d');
    
    timer_waitMillis(40000);

    */
    oi_t *robot = oi_alloc();
    while (1) {
        lcd_printf(robot->cliffLeft);
    }

}
