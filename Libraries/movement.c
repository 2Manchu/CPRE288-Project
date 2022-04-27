//
// Created by tonym on 4/22/2022.
//

#include "movement.h"
#include "uart-interrupt.h"

#define LEFT_TURN_OFFSET 12
#define RIGHT_TURN_OFFSET 14

int move_forward(oi_t *sensor_data, int distance_mm) {
    char str[50] = {'\0'};
    sprintf(str, "!GOING FORWARD %d cm\r\n", distance_mm / 10);
    uart_sendStr(str);
    oi_setWheels(150, 150);
    int sum = 0;

    while (sum < distance_mm) {
        oi_update(sensor_data);
        sum += sensor_data->distance;

        if (sensor_data->bumpLeft) {
            uart_sendStr("!LEFT BUMP DETECTED\r\n");
            oi_setWheels(0,0);
            move_backward(sensor_data, 150);
            return 1;
        }
        else if (sensor_data->bumpRight) {
            uart_sendStr("!RIGHT BUMP DETECTED\r\n");
            oi_setWheels(0,0);
            move_backward(sensor_data, 150);
            return 2;
        }
        //If we have a left sensor detection
        else if (sensor_data->cliffFrontLeftSignal > 2500 || sensor_data->cliffFrontLeftSignal < 500 ||
                 sensor_data->cliffLeftSignal > 2500 || sensor_data->cliffLeftSignal < 500) {
            uart_sendStr("!LEFT CLIFF/BOUND DETECTED\r\n");
            oi_setWheels(0,0);
            move_backward(sensor_data, 150);
            return 4;
        }
        //If we have a right sensor detection
        else if(sensor_data->cliffFrontRightSignal > 2500 || sensor_data->cliffFrontRightSignal < 500 ||
                sensor_data->cliffRightSignal > 2500 || sensor_data->cliffRightSignal < 500) {
            uart_sendStr("!RIGHT CLIFF/BOUND DETECTED\r\n");
            oi_setWheels(0,0);
            move_backward(sensor_data, 150);
            return 5;
        }
    }
    oi_setWheels(0,0);
    return 0;
}

void move_backward(oi_t *sensor_data, int distance_mm) {
    char str[50] = {'\0'};
    sprintf(str, "!GOING BACKWARD %d cm\r\n", distance_mm / 10);
    uart_sendStr(str);

    oi_setWheels(-175,-175);
    int sum = distance_mm;

    while (sum > 0) {
        oi_update(sensor_data);
        sum += sensor_data->distance;

        //Slight mods to make sure we stop if we're about to go out of bounds when going backward
        if (sensor_data->cliffFrontLeftSignal > 2500 || sensor_data->cliffFrontLeftSignal < 500 ||
                 sensor_data->cliffLeftSignal > 2500 || sensor_data->cliffLeftSignal < 500) {
            uart_sendStr("!LEFT CLIFF/BOUND DETECTED\r\n");
            oi_setWheels(0,0);
        }
            //If we have a right sensor detection
        else if(sensor_data->cliffFrontRightSignal > 2500 || sensor_data->cliffFrontRightSignal < 500 ||
                sensor_data->cliffRightSignal > 2500 || sensor_data->cliffRightSignal < 500) {
            uart_sendStr("!RIGHT CLIFF/BOUND DETECTED\r\n");
            oi_setWheels(0,0);
        }
    }

    oi_setWheels(0, 0);
}

int turnLeftAngle(oi_t *sensor_data, int angleToTurnTo) {
    char str[50] = {'\0'};
    sprintf(str, "!TURNING LEFT %d degrees\r\n", angleToTurnTo);
    uart_sendStr(str);

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
    char str[50] = {'\0'};
    sprintf(str, "!TURNING RIGHT %d degrees\r\n", angleToTurnTo);
    uart_sendStr(str);

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
