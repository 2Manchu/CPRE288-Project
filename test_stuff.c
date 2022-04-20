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

#define LEFT_TURN_OFFSET 12
#define RIGHT_TURN_OFFSET 14
#define IR_THRESHOLD_VAL 850

int move_forward(oi_t *sensor_data, int distance_mm) {
    //TODO: ADD CODE TO DETECT IF WE ARE DROPPING OR ON THE TAPE
    oi_setWheels(150, 150);
    int sum = 0;

    while (sum < distance_mm) {
        oi_update(sensor_data);

        lcd_printf("Left Signal: %d\n Right Signal: %d", sensor_data->cliffLeftSignal, sensor_data->cliffRightSignal);

        sum += sensor_data->distance;

        if (sensor_data->bumpLeft) {
            oi_setWheels(0,0);
            return 1;
        }
        else if (sensor_data->bumpRight) {
            oi_setWheels(0,0);
            return 2;
        }
        else if (sensor_data->bumpRight && sensor_data->bumpLeft) {
            oi_setWheels(0,0);
            return 3;
        }
            //If we've hit the white tape or cliff with left sensors
        else if (sensor_data->cliffFrontLeftSignal > 2500 || sensor_data->cliffFrontLeftSignal < 500 ||
                 sensor_data->cliffLeftSignal > 2500 || sensor_data->cliffLeftSignal < 500) {
            oi_setWheels(0,0);
            return 4;
        }
            //If we've hit the white tape or cliff with right sensors
        else if(sensor_data->cliffFrontRightSignal > 2500 || sensor_data->cliffFrontRightSignal < 500 ||
                sensor_data->cliffRightSignal > 2500 || sensor_data->cliffRightSignal < 500) {
            oi_setWheels(0,0);
            return 5;
        }
    }
    return 0;
}
void move_backward(oi_t *sensor_data, int distance_mm) {
    oi_setWheels(-175,-175);
    int sum = distance_mm;

    while (sum > 0) {
        oi_update(sensor_data);
        lcd_printf("Left Signal: %d\n Right Signal: %d", sensor_data->cliffLeftSignal, sensor_data->cliffRightSignal);
        sum += sensor_data->distance;
    }

    oi_setWheels(0, 0);
}

int turnLeftAngle(oi_t *sensor_data, int angleToTurnTo) {
    double sum = 0;
    oi_setWheels(100, -100);

    //If we have an object that's closer than whatever our left turn angular offset is, just turn left to half of the offset degrees
    if (angleToTurnTo - LEFT_TURN_OFFSET <= 0) {
        while (sum < LEFT_TURN_OFFSET / 2) {
            oi_update(sensor_data);
            sum += sensor_data->angle;
        }
        oi_setWheels(0, 0);
        return -1;
    }

    while (sum < angleToTurnTo - LEFT_TURN_OFFSET) {
        oi_update(sensor_data);
        sum += sensor_data->angle;
    }
    oi_setWheels(0, 0);
    return 0;
}

int turnRightAngle(oi_t *sensor_data, int angleToTurnTo) {

    double sum = 0;
    int corrAngle = angleToTurnTo + RIGHT_TURN_OFFSET;

    //If our angle to turn right is less than the offset, then just turn right amount of offset divided by 2
    if (corrAngle >= 0) {
        oi_setWheels(-100, 100);
        while(sum > RIGHT_TURN_OFFSET / 2) {
            oi_update(sensor_data);
            sum += sensor_data->angle;
        }
        oi_setWheels(0, 0);
        return -1;
    }

    oi_setWheels(-100, 100);
    while(sum > corrAngle) {
        oi_update(sensor_data);
        sum += sensor_data->angle;
    }
    oi_setWheels(0,0);
    return 0;
}

void main() {


    int i;
    timer_init();
    lcd_init();
    adc_init();
    button_init();
    ping_init();
    servo_init();
    uart_interrupt_init();

    oi_t *robot = oi_alloc();
    oi_init(robot);
    oi_update(robot);

    int moveValue = move_forward(robot, 1000);
    lcd_printf("%d", moveValue);
    if(moveValue == 1) {

    }
    else if(moveValue == 2) {

    }
    else if(moveValue == 3) {

    }
    else if(moveValue == 4) {
        move_backward(robot, 200);
        turnRightAngle(robot, -90);
        move_forward(robot, 500);

    }
    else if(moveValue == 5) {
        move_backward(robot, 200);
        turnLeftAngle(robot, 90);
        move_forward(robot, 500);
    }


    oi_free(robot);

}
