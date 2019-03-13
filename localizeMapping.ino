#include <SonarLib.h>
#include <MapManagerLib.h>

#define LeftTRIG 11
#define LeftECHO 12
#define RightTRIG 11
#define RightECHO 12

struct Direction {
    bool isFacingX;
    bool isForward;
}

struct Position {
    int x;
    int y;
    Direction direction;
};

const int squareDistance = 30;

Position current;
SonarLib leftSonar;
SonarLib rightSonar;

MapManagerLib mapManager;


void setup() {
    current.x = 0;
    current.y = 0;
    current.direction.isFacingX = false;
    current.direction.isForward = true;

    leftSonar = SonarLib(LeftTRIG, LeftECHO);
    rightSonar = SonarLib(RightTRIG, RightECHO);

    Serial.begin(9600);
}
 
void loop() {
    
}

// Drives robot to given location
// Perhaps this function should go into the motor lib
void driveToLocation(int targetX, int targetY) {
    if (current.direction.isFacingX) {
        bool isFacingWrongDirection = (current.direction.isForward) ? current.x > targetX : current.x < targetX;
        if (isFacingWrongDirection && current.y == targetY) {
            // y is correct but facing wrong dir
            // motors.turnAngle(180);
        } else if (isFacingWrongDirection) {
            if (current.y > targetY) {
                // turn 90 CCW
            } else {
                // turn 90 CW
            }
        }
    } else {
        bool isFacingWrongDirection = (current.direction.isForward) ? current.y > targetY : current.y < targetY;
        if (isFacingWrongDirection && current.x == targetX) {
            // y is correct but facing wrong dir
            // motors.turnAngle(180);
        } else if (isFacingWrongDirection) {
            if (current.x > targetX) {
                // turn 90 CCW
            } else {
                // turn 90 CW
            }
        }
    }

    // motors.driveForward()

    // Drive until inline with target location
    while (current.x != targetX && current.y != targetY) {
        // update current location from accelerometer
    }

    if (current.direction.isFacingX) {
        if (current.y > targetY) {
            // turn 90 CCW
        } else if (current.y < targetY) {
            // turn 90 CW
        }
    } else {
        if (current.x > targetX) {
            // turn 90 CCW
        } else if (current.x < targetX) {
            // turn 90 CW
        }
    }

    // motors.driveForward()

    // Drive until inline with target location
    while (current.x != targetX && current.y != targetY) {
        // update current location from accelerometer
    }

    // motors.stop()
}

// Uses the side sonars to update the map
// Assuming that the robot is on GROUND
void mapCurrentLocation() {
    // Set current location to be GROUND
    mapManager.setMapValue(current.x, current.y, mapManager.GROUND);

    // Get distances from sonars
    float rightDistance = rightSonar.getDistance();
    float leftDistance = leftSonar.getDistance();

    // Find number of squares before something has been detected
    // This is assuming the given distance from sonar reaches the end
    // of the last square
    int rightSquares = floor(rightDistance / squareDistance);
    int leftSquares = floor(leftDistance / squareDistance);

    if (current.direction.isFacingX) {
        // Facing x dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares;
        if (current.direction.isForward) {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
        } else {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
        }

        // Loop over all squares on each side setting them to PROPABLY_GROUND
        // Since sonar passes over all the squares we know that there is a large
        // possibly that they are GROUND, but this could be false, eg on the side there
        // are 2 or more water squares, so we would be incorrectly setting water squars to GROUND
        // After loop sent the last square to ITEM == something here
        // The last square is not guaranteed to be an objective, eg could be water square
        for (int i = current.y + 1; i < current.y + positiveSquares; i++) {
            mapManager.setMapValue(current.x, i, mapManager.PROPABLY_GROUND);
        }
        mapManager.setMapValue(current.x, current.y + positiveSquares, mapManager.ITEM);
        for (int i = current.y - 1; i > current.y - negativeSquares; i--) {
            mapManager.setMapValue(current.x, i, mapManager.PROPABLY_GROUND);
        }
        mapManager.setMapValue(current.x, current.y - negativeSquares, mapManager.ITEM);
    } else {
        // Facing y dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares;
        if (current.direction.isForward) {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
        } else {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
        }

        // Loop over all squares on each side setting them to PROPABLY_GROUND
        // Since sonar passes over all the squares we know that there is a large
        // possibly that they are GROUND, but this could be false, eg on the side there
        // are 2 or more water squares, so we would be incorrectly setting water squars to GROUND
        // After loop sent the last square to ITEM == something here
        // The last square is not guaranteed to be an objective, eg could be water square
        for (int i = current.x + 1; i < current.x + positiveSquares; i++) {
            mapManager.setMapValue(i, current.y, mapManager.PROPABLY_GROUND);
        }
        mapManager.setMapValue(current.x + positiveSquares, current.y, mapManager.ITEM);
        for (int i = current.x - 1; i > current.x - negativeSquares; i--) {
            mapManager.setMapValue(i, current.y, mapManager.PROPABLY_GROUND);
        }
        mapManager.setMapValue(current.x - negativeSquares, current.y, mapManager.ITEM);
    }
}
