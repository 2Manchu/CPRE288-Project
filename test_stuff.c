//
// Created by Tony Manschula on 4/13/22.
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

#define IR_THRESHOLD_VAL 600

/*
 * Holds data points from sensor scan.
 * Data fields:
 *      If dataPoints[x][] is data for angle x, dataPoints[x][0] contains PING distance, and
 *      dataPoints[x][1] contains raw IR sensor readings, and
 *      dataPoints[x][2] contains converted IR distance in cm.
 */
int dataPoints[181][3];

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
int skinnyPostFound = -1;

void eraseObjects() {
    //Currently hardcoded for size of objects array, adjust for size of array
    int i, j;
    for(i = 0; i < 15; ++i) {
        for(j = 0; j < 4; ++j) {
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

        //Send the IR raw reading to putty
        char ir[5] = { '\0' };
        int irRaw = scan.irRaw;
        dataPoints[currAngle][1] = irRaw;
        sprintf(ir, "%d", irRaw);
        for (i = 0; i < 5; i++) {
            uart_sendChar(ir[i]);
        }
        uart_sendChar('\t');
        uart_sendChar('\t');
        uart_sendChar('\t');

        dataPoints[currAngle][2] = (int)scan.irDist;
        char irD[5] = {'\0'};
        sprintf(irD, "%lf", scan.irDist);
        for (i = 0; i < 5; i++) {
            uart_sendChar(irD[i]);
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
            //TODO Change 10 to the width of the skinny posts
            //TODO: DO SOME TESTING TO FIND THE SIZE OF SKINNY BOIS
            if(objects[objNum][2] <= 9) {
                skinnyPostFound = objNum;
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
    int i;
    for (i = 0; i < numObjs - 1; ++i) {
        //Angular position to center of gap
        gaps[i][1] = (objects[i][0] + objects[i + 1][0]) / 2;

        //Angular width of gap
        int angularWidth = objects[i + 1][0] - objects[i][0];

        //Linear width of gap
        int distToSmallestObj;
        //We use the distance to the closer object to gauge the width of gap
        if (objects[i][1] > objects[i + 1][1]) {
            distToSmallestObj = objects[i + 1][1];
        } else {
            distToSmallestObj = objects[i][1];
        }
        gaps[i][0] = 2 * distToSmallestObj * sin(angularWidth / 2);
    }
    return i;
}


int findSkinnyPost(int numObjects) {
    int skinnyNum = -1;
    int i = 0;
    for(i = 0; i < numObjects; ++i) {

        //Width and bounds are in mm
        int currObjWidth = objects[i][2];
        int lowBound = 1;
        int highBound = currObjWidth + 6;

        //Check if the current object's width is within the bounds of the correct width
        if(currObjWidth > lowBound && currObjWidth < highBound) {
            skinnyNum = i;
            break;
        }
        return skinnyNum;
    }
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

    //servo_calibrate();
    set_left(37200);
    set_right(9400);

    scanInstance scan;

//    for (i = 0; i <= 180; i = i + 2) {
//        doScan(i, &scan);
//        lcd_printf("Angle: %d\nIR Raw: %d\nIR Dist: %.3lf\nPING Dist: %f", scan.angle, scan.irRaw, scan.irDist, scan.pingDist);
//    }

    scanSweep(scan);
    int numObjects = findObjects(scan);
    int numGaps = findGaps(numObjects);
    if(skinnyPostFound != -1) {
        int skinnyNum = skinnyPostFound;
        int skinnyAngle = objects[skinnyNum][0];
        int skinnyDistance = objects[skinnyNum][1];
        int skinnyWidth = objects[skinnyNum][2];
        if(skinnyAngle < 90) {
            turnRightAngle(robot, skinnyAngle - 90);
        }
        else if(skinnyAngle > 90) {
            turnLeftAngle(robot, skinnyAngle - 90);
        }
        move_forward(robot, skinnyDistance - 50);
        lcd_printf("Skinny Number: %d\nSkinny Width: %d\nnumObjects: %d\nnumGaps: %d", skinnyNum, skinnyWidth, numObjects, numGaps);
    }
    else {
        lcd_printf("numObjects: %d\nnumGaps: %d", numObjects, numGaps);
    }

    oi_free(robot);

}
