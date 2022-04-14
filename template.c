//
// Created by Tony Manschula,  on 2/9/22.
//

#include "stdio.h"
#include "lcd.h"
#include "Timer.h"
#include "uart-interrupt.h"
#include "adc.h"
#include "button.h"
#include "open_interface.h"
#include "ping.h"
#include "servo.h"
#include "scan.h"

#define LEFT_TURN_OFFSET 12
#define RIGHT_TURN_OFFSET 14
#define IR_THRESHOLD_VAL 850

/*
 * Holds data points from sensor scan.
 * Data fields:
 *      If dataPoints[x][] is data for angle x, dataPoints[x][0] contains PING distance, and
 *      dataPoints[x][1] contains IR sensor readings
 */
int dataPoints[181][2];

/*
 * Dimension 1 stores object number. Dimension 2 contains angular position of object, distance to object, linear width, and radial width respectively.
 */
int objects[15][4] = { '\0' };

int move_forward(oi_t *sensor_data, int distance_mm) {
    //TODO: ADD CODE TO DETECT IF WE ARE DROPPING OR ON THE TAPE
    oi_setWheels(150, 150);
    int sum = 0;

    while (sum < distance_mm) {
        oi_update(sensor_data);
        sum += sensor_data->distance;

        if (sensor_data->bumpLeft) {
            return 1;
        }
        else if (sensor_data->bumpRight) {
            return 2;
        }
        else if (sensor_data->bumpRight && sensor_data->bumpLeft) {
            return 3;
        }
    }
    oi_setWheels(0,0);
    return 0;
}


void move_backward(oi_t *sensor_data, int distance_mm) {
    oi_setWheels(-175,-175);
    int sum = distance_mm;

    while (sum > 0) {
        oi_update(sensor_data);
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

void updateDisplay(char display[], char keyPress) {
    sprintf(display, "Key Pressed: %c", keyPress);
    lcd_printf(display);
}

void scanSweep(scanInstance scan) {
    //Scan Angle Range
    doScan(0, &scan);
    timer_waitMillis(1500);
    int currAngle;
    //Make a 180 degree sweep of the field
    for (currAngle = 0; currAngle <= 180; currAngle += 2) {
        int i;
        doScan(currAngle, &scan);

        //Send the angle we just scanned to putty
        char angle[4] = { '\0' };
        sprintf(angle, "%d", currAngle);
        for (i = 0; i < 4; i++) {
            uart_sendChar(angle[i]);
        }
        uart_sendChar('\t');
        uart_sendChar('\t');

        //Send the initial ping distance to putty
        char ping[5] = { '\0' };
        float pingDist = scan.pingDist;
        //Read PING distance into data points
        dataPoints[currAngle][0] = pingDist;
        sprintf(ping, "%f", pingDist);
        for (i = 0; i < 6; i++) {
            uart_sendChar(ping[i]);
        }
        uart_sendChar('\t');
        uart_sendChar('\t');
        uart_sendChar('\t');

        //Send the IR distance to putty
        char ir[5] = { '\0' };
        int irDist = scan.irRaw;
        dataPoints[currAngle][1] = irDist;
        sprintf(ir, "%d", irDist);
        for (i = 0; i < 5; i++) {
            uart_sendChar(ir[i]);
        }
        uart_sendChar('\r');
        uart_sendChar('\n');
    }
}


int findObjects(scanInstance scan) {
    int objectStartDeg, objectEndDeg, angularWidth;
    double arcLength;
    int i, j;
    double sum = 0;
    int objNum = 0;

    //Go through the data points to find objects
    //Set to 171 because the sensor on cybot 1 sucks ass
    for (i = 0; i < 181; i += 2) {
        int isObjFound = 0;
        //If the object is closer than our specified distance threshold, set that angle as the start of the object
        if (dataPoints[i][1] > IR_THRESHOLD_VAL) {
            objectStartDeg = i;
            isObjFound = 1;
        }

        //Go advance through data points until we find a value that is less than our distance threshold. This marks the end of an object
        while (dataPoints[i][1] > IR_THRESHOLD_VAL) {
            objectEndDeg = i + 2;
            i += 2;
        }
        if (isObjFound) {
            //Find angular position of the middle of the detected object
            objects[objNum][0] = (objectStartDeg + objectEndDeg - 2) / 2;
            angularWidth = objectEndDeg - objectStartDeg;
            objects[objNum][3] = angularWidth;
            //Find linear width using arc length as a pretty reasonable approximation. We could find chord length using arc len for an exact reading if arc len is not good enough

            int radius = (dataPoints[objectStartDeg][0] + dataPoints[objectEndDeg][0]) / 2;
            arcLength = 2.0 * M_PI * (double)radius * ((double)angularWidth / 360.0);
            objects[objNum][2] = (int)arcLength;

            objNum++;
            i += 2;
        }
    }

    //IMPROVING THE OBJECT DETECTION
    //Take an average of distance readings to objects that we've identified since ping sensor is not consistent
    for (i = 0; i < objNum; i++) {
        doScan(objects[i][0], &scan);
        objects[i][1] = scan.irDist;

        //If we get a bad distance do the scan again
        if (scan.irDist > 100) {
            i--;
        }

    }

    //Send out info to putty regarding the detected objects
    char angle[4] = { '\0' };
    char irDist[5] = { '\0' };
    char linWidth[3] = { '\0' };
    char header[39] = "AnglePos\tPING Distance\t\tLinear Width\r\n";

    for (i = 0; i <= 38; i++) {
        uart_sendChar(header[i]);
    }

    for (j = 0; j < objNum; j++) {
        if (objects[j][3] > 4) {
            sprintf(angle, "%d", objects[j][0]);
            sprintf(irDist, "%d", objects[j][1]);
            sprintf(linWidth, "%d", objects[j][2]);

            for (i = 0; i < 4; i++) {
                uart_sendChar(angle[i]);
            }
            uart_sendChar('\t');
            uart_sendChar('\t');

            for (i = 0; i < 5; i++) {
                uart_sendChar(irDist[i]);
            }
            uart_sendChar('\t');
            uart_sendChar('\t');
            uart_sendChar('\t');

            for (i = 0; i < 3; i++) {
                uart_sendChar(linWidth[i]);
            }
            uart_sendChar('\r');
            uart_sendChar('\n');
        }
    }
    return objNum;
}

void main() {
    timer_init();
    lcd_init();
    adc_init();
    button_init();
    ping_init();
    servo_init();
    uart_interrupt_init();

    //Servo calibration actions, uncomment when doing new robot
    //servo_calibrate();
    set_left(35600);
    set_right(7500);

    //Create an open interface object
    oi_t *robot = oi_alloc();
    //Initialize it
    oi_init(robot);

    scanInstance scan;

    char display[21];

    //Print the table header for the initial sweep of the field
    char header[39] = "Degrees\t\tPING Distance (cm)\tIR Value\r\n";
    int i;
    for (i = 0; i <= 38; i++) {
        uart_sendChar(header[i]);
    }

    scanSweep(scan);
    int numObjs = findObjects(scan);

    #pragma clang diagnostic push
    #pragma ide diagnostic ignored "EndlessLoop"
    while(1) {
        //TODO: NAVIGATE BETWEEN OBJECTS, AND GO FORWARD IF NONE FOUND. ADD APPROPRIATE ACTIONS IF WE HIT A LINE OR CLIFF
        //Adjust position of object relative to robot, negative is to right, positive to left
        int objPos = objects[smallestObj][0] - 90;
        //if object is to the left
        if (objPos > 0)  {
            turnLeftAngle(robot, objPos);
        }
        //if object is to the right
        else if (objPos < 0 ) {
            turnRightAngle(robot, objPos);
        }

        int distanceToMove = (objects[smallestObj][1]*10);  //TODO: previously this was - 120 but want to see if the fixed average code eliminates the need for this
        int moveStatus = move_forward(robot, distanceToMove);
        int isAvoiding = 0;

        while (1) {
            if (!isAvoiding && moveStatus == 0)  {
                goto STOPWAITAMINUTE;
            }
            if (isAvoiding && moveStatus == 0) {
                isAvoiding = 0;
                scanSweep(scan);
                numObjs = findObjects(scan);
                smallestObj = findSmallestObject(numObjs);
                //(OLD) There's an offset here since the ping sensor seems to overestimate distance quite significantly
                distanceToMove = (objects[smallestObj][1]*10);
                objPos = objects[smallestObj][0] - 90;

                if (objPos > 0)  {
                    turnLeftAngle(robot, objPos);
                }
                //if object is to the right
                else if (objPos < 0 ) {
                    turnRightAngle(robot, objPos);
                }

                moveStatus = move_forward(robot, distanceToMove);
            }
            //Robot hit something on the left
            while (moveStatus == 1) {
                isAvoiding = 1;
                move_backward(robot, 30);
                //Negative value for turning since that's the way I programmed it
                turnRightAngle(robot, -65);
                moveStatus = move_forward(robot, 245);
                if (moveStatus == 0) {
                    turnLeftAngle(robot, 80);
                }
            }
            //The robot hit something on the right
            while (moveStatus == 2) {
                isAvoiding = 1;
                move_backward(robot, 37);
                turnLeftAngle(robot, 65);
                moveStatus = move_forward(robot, 245);
                //Negative value for turning since that's the way I programmed it
                if (moveStatus == 0) {
                    turnRightAngle(robot, 80);
                }
            }
            //Hit something head on
            while (moveStatus == 3) {
                isAvoiding = 1;
                move_backward(robot, 45);
                moveStatus == 0;
            }

        }


    }

    STOPWAITAMINUTE:
    //Stop robot, show exit indication, quit OI
    oi_setWheels(0, 0);
    updateDisplay(display, '!');
    oi_free(robot);
}
#pragma clang diagnostic pop
