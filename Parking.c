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

#define IR_THRESHOLD_VAL 675
#define LEFT_TURN_OFFSET 0
#define RIGHT_TURN_OFFSET 0
#define ROBOT_WIDTH 35

/*
 * Holds data points from sensor scan.
 * Data fields:
 *      If dataPoints[x][] is data for angle x, dataPoints[x][0] contains PING distance, and
 *      dataPoints[x][1] contains IR sensor readings
 */
int dataPoints[181][2];

/*
 * Dimension 1 stores object number. Dimension 2 contains angular position of object, distance to object, linear width, and angular width respectively.
 */
int objects[15][4] = { '\0' };

/**
 * Dimension 1 stores gap number. Dimension 2 contains gap width and angular position of the center of the gap, and the distance to the gap
 */
int gaps[14][3] = {'\0'};

/*
 * This variable is only used when we find the parking zone. It keeps track of the locations of all the skinny objects surrounding the zone
 * The first dimension is the number of the skinny object. The second stores its angular position relative to the robot and the distance to the skinny
 */
int skinnyObjects[4][2];

/*
 * -1 if there is no skinny post in range of scan
 * 1 if there is a skinny post
 */
int skinnyPostFound = -1;

int skinnyIndex = 0;

/*
 * Below are some simple functions to clear arrays. They should be self-explanatory
 */
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

void scanSweep(scanInstance scan) {
    //Scan Angle Range
    doScan(0, &scan);
    timer_waitMillis(1500);
    int currAngle;

    uart_sendStr("!Degrees\t\tPING Distance (cm)\tIR Value\r\n");

    //Make a 180 degree sweep of the field
    for (currAngle = 0; currAngle <= 180; currAngle += 2) {
        int i;
        doScan(currAngle, &scan);

        float pingDist = scan.pingDist;
        //Read PING distance into data points
        dataPoints[currAngle][0] = pingDist;

        //Read raw IR value into data points
        int irDist = scan.irRaw;
        dataPoints[currAngle][1] = irDist;

        //Only in we're in manual mode, send the data from each angle scanned to the terminal
        if (manualMode == 1) {
            //Send the angle we just scanned to putty
            char angle[4] = {'\0'};
            sprintf(angle, "%d", currAngle);
            for (i = 0; i < 4; i++) {
                uart_sendChar(angle[i]);
            }
            uart_sendChar('\t');
            uart_sendChar('\t');

            //Send the initial ping distance to putty
            char ping[5] = {'\0'};

            sprintf(ping, "%f", pingDist);
            for (i = 0; i < 6; i++) {
                uart_sendChar(ping[i]);
            }
            uart_sendChar('\t');
            uart_sendChar('\t');
            uart_sendChar('\t');

            //Send the IR distance to putty
            char ir[5] = {'\0'};

            sprintf(ir, "%d", irDist);
            for (i = 0; i < 5; i++) {
                uart_sendChar(ir[i]);
            }
            uart_sendChar('\r');
            uart_sendChar('\n');
        }
    }
}

/**
 * Find objects from the most recent scan, given a raw IR threshold value
 */
int findObjects(scanInstance scan, oi_t *robot) {
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

        //Make sure we don't include any <4 degree objects as these are fake and will falsely trigger the end zone detection.
        if (objectEndDeg - objectStartDeg <= 4) {
            isObjFound = 0;
        }

        if (isObjFound) {
            //Find angular position of the middle of the detected object
            int objAngPos = (objectStartDeg + objectEndDeg) / 2;
            if (objAngPos % 2 == 1) {
                objAngPos++;
            }
            objects[objNum][0] = objAngPos;
            //Putting the PING distance now (?!?!?!) from the middle angle of the object from datapoints array into the distance to object field in objects array
            int pingDistToObj = dataPoints[objects[objNum][0]][0];
            objects[objNum][1] = pingDistToObj;
            //Assign the object an angular width
            angularWidth = objectEndDeg - objectStartDeg;
            objects[objNum][3] = angularWidth;

            //Find linear width using arc length as a pretty reasonable approximation. We could find chord length using arc len for an exact reading if arc len is not good enough
            int radius = objects[objNum][1];
            arcLength = 2.0 * M_PI * (double)radius * ((double)angularWidth / 360.0);
            objects[objNum][2] = (int)arcLength;

            //If we've detected a skinny object, then assign it an angular position and a distance from robot
            if(objects[objNum][2] <= 9) {
                skinnyObjects[skinnyIndex][0] = objAngPos;
                skinnyObjects[skinnyIndex][1] = pingDistToObj;
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
    char header[39] = "AnglePos\tPG Distance\t\tLinear Width\r\n";

    uart_sendStr(header);

    //If the angular width of the object is greater than 4 degrees, then send the object's info to putty
    //Technically this check is unnecessary as we check to ensure that <4 degree objects are not included in the array in the first place
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

/**
 * Given an objects[][] array, find gaps in the objects as well as the positions, and widths
 * **HIGHLY EXPERIMENTAL**
 * @param numObjs How many objects are contained within the array from the current scan
 * @return An int corresponding to the number of gaps found
 */
int findGaps(int numObjs) {
    clearGaps();
    int i;
    uart_sendStr("Gap angle\tDist to gap\tGap lin width\r\n");
    for(i = 0; i < numObjs - 1; i++) {
        //Angular position to center of gap
        gaps[i][1] = (objects[i][0] + objects[i + 1][0]) / 2;

        //Angular width of gap
        int angularWidthGap = objects[i + 1][0] - objects[i][0];

        //Distance to gap
        int distToSmallestObj;
        //We use the distance to the closer object to gauge the width of gap
        if (objects[i][1] > objects[i + 1][1]) {
            distToSmallestObj = objects[i + 1][1];
            gaps[i][2] = distToSmallestObj;
        } else {
            distToSmallestObj = objects[i][1];
            gaps[i][2] = distToSmallestObj;
        }

        //Linear width of gap
        if (angularWidthGap < 90) {
            gaps[i][0] = distToSmallestObj * angularWidthGap * (M_PI / 180);
        }
        else {
            gaps[i][0] = distToSmallestObj * angularWidthGap * (M_PI / 180);
        }
        uart_sendStr("\r\n");

        char str[50] = {'\0'};
        sprintf(str, "%d\t\t%d\t\t%d\r\n", gaps[i][1], distToSmallestObj, gaps[i][0]);
        uart_sendStr(str);
    }

    return i;
}

/**
 * Function that finds the index of the closest gap in gaps[][]
 * @param numGaps The number of gaps within gaps array
 * @return An int of the index of the closest gap
 */
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

/**
 * Main function containing autonomous and manual control
 * **PLEASE NOTE:**
 * We demoed in full manual mode because autonomous just wasn't there in terms of polish.
 * Autonomous code was still present but just not utilized.
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
    //Current for CYBOT 8
    //servo_calibrate();
    set_left(35700);
    set_right(8300);

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

    #pragma clang diagnostic push
    #pragma ide diagnostic ignored "EndlessLoop"
        //Repeatedly scan, find objects, and move forward accordingly. If bump sensors are triggered, stop, back
        //up, and turn, but don't move forward. Repeat sequence and essentially bounce around testing area until
        //cliff sensors pick up blue tape that mark the end zone, which will then trigger the parking sequence
    while (1) {
        //removed skinnyPostFound == -1
        while (goCmd && !manualMode) {
            int numGaps = 0;
            int closestGap = -1;
            int distToGap = -1;
            int gapAngPos = -1;

            //Do initial scan
            scanSweep(scan);
            int numObjs = findObjects(scan, robot);
            //Realize that this condition never gets checked if we have skinnyPostFound == -1 as the outer while condition
            if (skinnyPostFound != -1) {
                uart_sendStr("!PARKING SEQUENCE INITIATED\r\n");
                break;
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
                if (gapAngPos > 0) {
                    turnLeftAngle(robot, gapAngPos);
                }
                else if (gapAngPos < 0 ) {
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

            //Code for object avoidance after the above actions are taken
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

        /*
         * This is manual mode, which we used to complete the demo
         */
        while (goCmd && manualMode) {
            if (manualMode == 0) {
                uart_sendStr("!ENTERING AUTONOMOUS MODE\r\n");
                break;
            }
            //Forward
            if (movementCode == 1) {
                move_forward(robot, 100);
                movementCode = 0;
            }
            //Backward
            else if (movementCode == 2) {
                //do a jank backwards thing spin around, move forward, then spin back to avoid holes
                turnLeftAngle(robot, 180 + LEFT_TURN_OFFSET);
                move_forward(robot, 100);
                turnRightAngle(robot, -180 + RIGHT_TURN_OFFSET);
                movementCode = 0;

            }
            //Left a little
            else if (movementCode == 3) {
                turnLeftAngle(robot, 10 + LEFT_TURN_OFFSET);
                movementCode = 0;
            }
            //Right a little
            else if (movementCode == 4) {
                turnRightAngle(robot, -10 + RIGHT_TURN_OFFSET);
                movementCode = 0;
            }
            //Scan
            else if (movementCode == 5) {
                scanSweep(scan);
                int numObjects = findObjects(scan, robot);
                findGaps(numObjects);
                movementCode = 0;
            }
            //Left 90
            else if(movementCode == 6) {
                turnLeftAngle(robot, 90 + LEFT_TURN_OFFSET);
                movementCode = 0;
            }
            //Right 90
            else if(movementCode == 7) {
                turnRightAngle(robot, -90 + RIGHT_TURN_OFFSET);
                movementCode = 0;
            }
            //Turn around
            else if(movementCode == 8) {
                turnLeftAngle(robot, 180 + LEFT_TURN_OFFSET);
                movementCode = 0;
            }

        }

        //removed skinnyPostFound != -1
        //Our parking zone detected logic
        //We encountered major issues getting this to work, which is the primary reason we did not do autonomous for the demo
        while (goCmd && skinnyPostFound) {
            //Start by running a scan. If we see skinny objects, they will be logged into the skinnyObjects array.
            //Go through that and see how many we have. If we have 1, go towards the object. If we have 2 then shoot the gap
            scanSweep(scan);
            int numObjs = findObjects(scan, robot);
            int howManySkinny = getNumSkinnys();
            //If we for some reason lose sight of the destination then return to normal mode
            if (howManySkinny == 0) {
                skinnyPostFound = -1;
                break;
            }
            //If 1 skinny then go towards it
            else if (howManySkinny == 1) {
                int turnAng = skinnyObjects[0][0] - 90;
                if (turnAng > 0) {
                    turnLeftAngle(robot, turnAng);
                }
                else if (turnAng < 0) {
                    turnRightAngle(robot, turnAng);
                }

                moveStatus = move_forward(robot, skinnyObjects[0][1] * 10);

                //Code for object avoidance after the above actions are taken
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
            //If more than 1 skinny, go between the first (rightmost) and the last (leftmost)
            else {
                int gapAng = skinnyObjects[skinnyIndex - 1][0] - skinnyObjects[0][0];
                int gapDist;

                if (skinnyObjects[skinnyIndex - 1][1] > skinnyObjects[0][0]) {
                    gapDist = skinnyObjects[skinnyIndex - 1][1];
                }
                else {
                    gapDist = skinnyObjects[0][0];
                }

                if (gapAng > 0) {
                    turnLeftAngle(robot, gapAng);
                }
                else if (gapAng < 0) {
                    turnRightAngle(robot, gapAng);
                }

                moveStatus = move_forward(robot, gapDist);

                //Code for object avoidance after the above actions are taken
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
                //TODO ADD AVOIDANCE LOGIC IN PARKING MODE
            }
        }

        //Emergency stop/program completed procedure
        if (goCmd == 0) {
            oi_setWheels(0, 0);
            lcd_printf("Program exited/\nDestination found");
            uart_sendStr("Exit/Destination found");
            oi_free(robot);
            break;
        }
    }
    oi_free(robot);
}
#pragma clang diagnostic pop
