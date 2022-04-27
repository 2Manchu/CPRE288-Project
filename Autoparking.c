//
// Created by Tony Manschula,  on 2/9/22.
//

#include "stdio.h"
#include "Libraries/lcd.h"
#include "Libraries/Timer.h"
#include "Libraries/uart-interrupt.h"
#include "Libraries/adc.h"
#include "Libraries/button.h"
#include "Libraries/open_interface.h"
#include "Libraries/ping.h"
#include "Libraries/servo.h"
#include "Libraries/scan.h"
#include "Libraries/movement.h"

#define IR_THRESHOLD_VAL 650
#define ROBOT_WIDTH 35

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

/**
 * Dimension 1 stores gap number. Dimension 2 contains gap width and angular position of the center of the gap, and the distance to the gap
 */
int gaps[14][3] = {'\0'};

/*
 * This variable is only used when we find the parking zone. It keeps track of the locations of all the skinny objects surrounding the zone
 * The first dimension is the number of the skinny object. The second stores its angular position relative to the robot
 */
int skinnyObjects[4][1];

/*
 * -1 if there is no skinny post in range of scan
 * 1 if there is a skinny post
 */
int skinnyPostFound = -1;

int skinnyIndex = 0;

void clearObjects() {
    int i, j;
    //Currently hardcoded for size of objects array, adjust for size of array
    for(i = 0; i < 15; ++i) {
        for(j = 0; j < 4; ++j) {
            objects[i][j] = '\0';
        }
    }
}

void clearGaps (void) {
    int i;
    int j;
    for (i = 0; i < 14; i++) {
        for (j = 0; j < 3; j++) {
            gaps[i][j] = '\0';
        }
    }
}

void clearSkinny (void) {
    int i;
    int j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 1; j++) {
            skinnyObjects[i][j] = '\0';
        }
    }
}

int getNumSkinnys() {
    int i = 0;
    while (skinnyObjects[i][0] != '\0') i++;

    return i;
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

    uart_sendStr("!Degrees\t\tPING Distance (cm)\tIR Value\r\n");

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
    clearObjects();
    clearSkinny();
    skinnyIndex = 0;
    int objectStartDeg, objectEndDeg, angularWidth;
    double arcLength;
    int i, j;
    double sum = 0;
    int objNum = 0;

    //Go through the data points to find objects
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

        //Make sure we don't include any 2 degree objects as these are fake and will falsely trigger the end zone detection.
        if (objectEndDeg - objectStartDeg <= 2) {
            isObjFound = 0;
            //Decrement i once since it will have been incorrectly incremented by the previous code since we didn't really have an object detected
            i -= 2;
        }

        if (isObjFound) {
            //Find angular position of the middle of the detected object
            int objAngPos = (objectStartDeg + objectEndDeg) / 2;
            if (objAngPos % 2 == 1) {
                objAngPos++;
            }
            objects[objNum][0] = objAngPos;
            //Putting the PING distance now (?!?!?!) from the middle angle of the object from datapoints array into the distance to object field in objects array
            objects[objNum][1] = dataPoints[objects[objNum][0]][0];
            //Assign the object an angular width
            angularWidth = objectEndDeg - objectStartDeg;
            objects[objNum][3] = angularWidth;

            //Find linear width using arc length as a pretty reasonable approximation. We could find chord length using arc len for an exact reading if arc len is not good enough
            int radius = objects[objNum][1];
            arcLength = 2.0 * M_PI * (double)radius * ((double)angularWidth / 360.0);
            objects[objNum][2] = (int)arcLength;
            //TODO Change number below to appropriate width of skinny
            if(objects[objNum][2] <= 9) {
                skinnyObjects[skinnyIndex][objAngPos];
                skinnyIndex++;
                skinnyPostFound = 1;
            }
            objNum++;
            i += 2;
        }
    }

    //Send out info to putty regarding the detected objects
    char angle[4] = { '\0' };
    char irDist[5] = { '\0' };
    char linWidth[3] = { '\0' };
    char header[39] = "AnglePos\tIR Distance\t\tLinear Width\r\n";

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

int findGaps(int numObjs) {
    clearGaps();
    int i;
    uart_sendStr("Gap angle\tDist to gap\tGap lin width\r\n");
    for(i = 0; i < numObjs - 1; i++) {
        //Angular position to center of gap
        gaps[i][1] = (objects[i][0] + objects[i + 1][0]) / 2;
        uart_sendStr((const char *) gaps[i][1]);
        uart_sendStr("\t\t");

        //Angular width of gap
        int angularWidth = objects[i + 1][0] - objects[i][0];

        //Distance to gap
        int distToSmallestObj;
        //We use the distance to the closer object to gauge the width of gap
        if (objects[i][1] > objects[i + 1][1]) {
            distToSmallestObj = objects[i + 1][1];
            gaps[i][2] = distToSmallestObj;
        }
        else {
            distToSmallestObj = objects[i][1];
            gaps[i][2] = distToSmallestObj;
        }
        uart_sendStr((const char *) distToSmallestObj);
        uart_sendStr("\t\t");
        //Linear width of gap
        gaps[i][0] = 2 * distToSmallestObj * sin(angularWidth / 2);
        uart_sendStr("\r\n");
    }

    return i;
}

int findClosestGap(int numGaps) {
    int i;
    int smallestGapNum;
    if (numGaps == 1) {
        return 0;
    }
    else if (numGaps == 2 && gaps[0][2] < gaps[1][2]) {
        return 0;
    }
    else if (numGaps == 2 && gaps[0][2] > gaps[1][2]) {
        return 1;
    }
    else {
        for (i = 0; i < numGaps - 1; i++) {
            if (gaps[i][2] < gaps[i + 1][2]) {
                smallestGapNum = i;
            }
            else {
                smallestGapNum = i + 1;
            }
        }
    }

}

/*
int findSkinnyPost(int numObjects) {
    int skinnyNum = -1;
    for(int i = 0; i < numObjects; ++i) {

        //Width and bounds are in mm
        int currObjWidth = objects[i][2] / 10;
        int lowBound = 55;
        int highBound = currObjWidth + 61;

        //Check if the current object's width is within the bounds of the correct width
        if(currObjWidth > lowBound && currObjWidth < highBound) {
            skinnyNum = i;
            break;
        }
        return skinnyNum;
    }
}
 */

void main() {
    timer_init();
    lcd_init();
    adc_init();
    button_init();
    ping_init();
    servo_init();
    uart_interrupt_init();

    //Servo calibration actions, uncomment when doing new robot
    //Current for CYBOT 12
    //servo_calibrate();
    set_left(37200);
    set_right(9400);

    //Create an open interface object
    oi_t *robot = oi_alloc();
    //Initialize it
    oi_init(robot);

    scanInstance scan;

    char display[21];
    int isAvoiding = 0;
    int moveStatus = -1;

    while (!goCmd) {};  //busywait on the command to go from the uart controller
    uart_sendStr("!STARTING SEQUENCE\r\n");

    //Print the table header for the initial sweep of the field
    //TODO: IF WE ARE DOING OUTPUT AS CSV WE'LL HAVE TO CHANGE THIS

    #pragma clang diagnostic push
    #pragma ide diagnostic ignored "EndlessLoop"
        //TODO: NAVIGATE BETWEEN OBJECTS, AND GO FORWARD IF NONE FOUND. ADD APPROPRIATE ACTIONS IF WE HIT A LINE OR CLIFF
        //TODO: If we see a skinny pillar in the findObjects, then we set a special return code in the findObjects function, and have a goto that initiates the parking sequence

        //Repeatedly scan, find objects, and move forward accordingly. If bump sensors are triggered, stop, back
        //up, and turn, but don't move forward. Repeat sequence and essentially bounce around testing area until
        //cliff sensors pick up blue tape that mark the end zone, which will then trigger the parking sequence
    while (1) {
        while (goCmd && !manualMode && skinnyPostFound == -1) {
            if (manualMode == 1) {
                uart_sendStr("!MANUAL MODE ENTERED\r\n");
                goto BOTTOM;
            }
            int numGaps = 0;
            int closestGap = -1;
            int distToGap = -1;
            int gapAngPos = -1;

            //Do initial scan
            scanSweep(scan);
            int numObjs = findObjects(scan);
            //TODO skinny post found sequence
            if (skinnyPostFound != -1) {
                uart_sendStr("!PARKING SEQUENCE INITIATED\r\n");
                goto BOTTOM;
            }

            //Special cases if there's no gaps to go
            if (numObjs == 0) {
                moveStatus = move_forward(robot, 350);
            }
            //If there's only one object just go towards it
            else if (numObjs == 1) {
                if (objects[0][0] <= 90) {
                    turnRightAngle(robot, -30);
                } else {
                    turnLeftAngle(robot, 30);
                }
                moveStatus = move_forward(robot, 350);
            }
            //We have several objects so go towards the closest gap
            else if (numObjs > 1) {
                numGaps = findGaps(numObjs);
                closestGap = findClosestGap(numGaps);

                gapAngPos = gaps[closestGap][1];
                gapAngPos = gapAngPos - 90;
                if (gapAngPos > 90) {
                    turnLeftAngle(robot, gapAngPos);
                }
                else if (gapAngPos < 90 ) {
                    turnRightAngle(robot, gapAngPos);
                }

                distToGap = gaps[closestGap][2];
                //Check to make sure we fit thru the gap
                if (gaps[closestGap][0] >= 35) {
                    moveStatus = move_forward(robot, (distToGap + 10) * 10);
                }
                //If not then we move forward a little bit and scan again
                else {
                    move_forward(robot, 100);
                }
            }

            if (moveStatus == 1) {
                turnRightAngle(robot, -90);
            }
            else if (moveStatus == 2) {
                turnLeftAngle(robot, 90);
            }
                //We have a left side cliff detection
            else if (moveStatus == 4) {
                turnRightAngle(robot, -90);
            }
                //We have a right side cliff detection
            else if (moveStatus == 5) {
                turnLeftAngle(robot, 90);
            }
        }

        while (goCmd && manualMode && skinnyPostFound == -1) {
            if (manualMode == 0) {
                uart_sendStr("!ENTERING AUTONOMOUS MODE\r\n");
                break;
            }
            if (movementCode == 1) {
                move_forward(robot, 100);
                movementCode = 0;
            } else if (movementCode == 2) {
                //do a jank backwards thing spin around, move forward, then spin back to avoid holes
                turnLeftAngle(robot, 180);
                move_forward(robot, 100);
                turnRightAngle(robot, 180);
                movementCode = 0;
            } else if (movementCode == 3) {
                turnLeftAngle(robot, 15);
                movementCode = 0;
            } else if (movementCode == 4) {
                turnRightAngle(robot, -15);
                movementCode = 0;
            }

        }

        //Our parking zone detected logic
        //IMPORTANT NOTE, IN THIS MODE THE
        while (goCmd && skinnyPostFound != -1) {
            //Start by running a scan. If we see skinny objects, they will be logged into the skinnyObjects array.
            //Go through that and see how many we have. If we have 1, go towards the object. If we have 2 then shoot the gap
        }

        //Emergency stop code
        if (goCmd == 0) {
            oi_setWheels(0, 0);
            updateDisplay(display, '!');
            oi_free(robot);
            break;
        }

        BOTTOM:
        continue;
    }
}
#pragma clang diagnostic pop
