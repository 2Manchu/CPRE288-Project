//
// Created by tonym on 4/22/2022.
//

#ifndef CPRE288_PROJECT_MOVEMENT_H
#define CPRE288_PROJECT_MOVEMENT_H

#include "open_interface.h"

int move_forward(oi_t *sensor_data, int distance_mm);

void move_backward(oi_t *sensor_data, int distance_mm);

int turnLeftAngle(oi_t *sensor_data, int angleToTurnTo);

int turnRightAngle(oi_t *sensor_data, int angleToTurnTo);


#endif //CPRE288_PROJECT_MOVEMENT_H
