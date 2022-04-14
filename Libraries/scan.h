//
// Created by tonym on 4/14/2022.
//

#ifndef CPRE288_PROJECT_SCAN_H
#define CPRE288_PROJECT_SCAN_H

#include "adc.h"
#include "ping.h"
#include "servo.h"
#include "Timer.h"

typedef struct {
    uint16_t irRaw;
    double irDist;
    float pingDist;
    int angle;
} scanInstance;

void doScan(int angle, scanInstance* scan);

#endif //CPRE288_PROJECT_SCAN_H

