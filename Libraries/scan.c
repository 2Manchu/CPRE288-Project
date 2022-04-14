//
// Created by tonym on 4/14/2022.
//

#include "scan.h"

void doScan(int angle, scanInstance* scan) {
    servo_move(angle);
    timer_waitMillis(150);
    scan->irRaw = adc_read();
    scan->irDist = adc_getDistance(scan->irRaw);
    scan->pingDist = ping_getDistance();
}