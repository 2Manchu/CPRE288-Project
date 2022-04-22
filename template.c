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
#include "movement.h"
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
 * Dimension 1 stores gap number. Dimension 2 contains gap width and angular position of the center of the gap
 */
int gaps[14][2] = {'\0'};

/*
 * 0 if there is no skinny post in range of scan
 * 1 if skinny post has been found
 */
int skinnyPostFound = 0;

void eraseObjects() {
    //Currently hardcoded for size of objects array, adjust for size of array
    for(int i = 0; i < 15; ++i) {
        for(int j = 0; j < 4; ++j) {
            objects[i][j] = '\0';
        }
    }
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
    eraseObjects();
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
            //TODO Change 10 to the width of the skinny posts
            //TODO: DO SOME TESTING TO FIND THE SIZE OF SKINNY BOIS
            if(objects[objNum][2] < 10) {
                skinnyPostFound = 1;
            }
            objNum++;
            i += 2;
        }
    }

    //IMPROVING THE OBJECT DETECTION
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

int findGaps(int numObjs) {
    int i;
    for(i = 0; i < numObjs - 1; ++i) {
        //Angular position to center of gap
        gaps[i][1] = (objects[i][0] + objects[i + 1][0]) / 2;

        //Angular width of gap
        int angularWidth = objects[i + 1][0] - objects[i][0];

        //Linear width of gap
        int distToSmallestObj;
        //We use the distance to the closer object to gauge the width of gap
        if (objects[i][1] > objects[i + 1][1]) {
            distToSmallestObj = objects[i + 1][1];
        }
        else {
            distToSmallestObj = objects[i][1];
        }
        gaps[i][0] = 2 * distToSmallestObj * sin(angularWidth / 2);
    }
    return i;
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
    int isAvoiding = 0;
    int moveStatus = -1;

    //Print the table header for the initial sweep of the field
    //TODO: IF WE ARE DOING OUTPUT AS CSV WE'LL HAVE TO CHANGE THIS
    char header[39] = "Degrees\t\tPING Distance (cm)\tIR Value\r\n";
    int i;
    for (i = 0; i <= 38; i++) {
        uart_sendChar(header[i]);
    }

    while (!goCmd) {};  //busywait on the command to go from the uart controller

    #pragma clang diagnostic push
    #pragma ide diagnostic ignored "EndlessLoop"
        //TODO: NAVIGATE BETWEEN OBJECTS, AND GO FORWARD IF NONE FOUND. ADD APPROPRIATE ACTIONS IF WE HIT A LINE OR CLIFF
        //TODO: If we see a skinny pillar in the findObjects, then we set a special return code in the findObjects function, and have a goto that initiates the parking sequence
        //TODO: FINISH THE LABELS MANUAL AND AUTONOMOUS TO ALLOW FOR OVERRIDE CONTROLS FROM THE BASE STATION

        //Repeatedly scan, find objects, and move forward accordingly. If bump sensors are triggered, stop, back
        //up, and turn, but don't move forward. Repeat sequence and essentially bounce around testing area until
        //cliff sensors pick up blue tape that mark the end zone, which will then trigger the parking sequence
    AUTONOMOUS_MODE:
        while (goCmd) {
            if (manualMode) goto MANUAL_MODE;
            //Adjust position of object relative to robot, negative is to right, positive to left
            eraseObjects();
            int numGaps = 0;
            scanSweep(scan);
            int numObjs = findObjects(scan);
            //TODO skinny post found sequence -- use goto
            if(skinnyPostFound == 1) {

            }

            //Special cases if there's no gaps to go
            if (numObjs == 0) {
                continue;
            }
            else if (numObjs == 1) {
                if (objects[0][0] <= 90) {
                    turnRightAngle(robot, -30);
                }
                else {
                    turnLeftAngle(robot, 30);
                }
            }
            else {
                numGaps = findGaps(numObjs);
            }
            //TODO: Move into gaps
            //After this we'll have to use the objects array and number of objects, plus a helper function to find the distance between the two objects
            //Then we find the angle to it using the above code snippet and get distance, then move to the gap
            //We should also ensure before moving that we fit through the gap by checking gaps[1] against our width
            moveStatus = move_forward(robot, 500);
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

    MANUAL_MODE:
    while (goCmd) {
        if (!manualMode) goto AUTONOMOUS_MODE;
        if ()
    }



    //Just added an if statement here to prevent it from executing this code when goCmd is no longer true.
    //This will occur when the user issues presses 'o' while the program is still in normal execution and the destination has not been found
    if (goCmd) {
        PARK:
        //Stop robot, show exit indication, quit OI
        oi_setWheels(0, 0);
        updateDisplay(display, '!');
        oi_free(robot);
    }
}
#pragma clang diagnostic pop
