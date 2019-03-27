#include "Arduino.h"
#include "MapManagerLib.h"

MapManagerLib::MapManagerLib() {
    // Setup known map
    MAP[0][0] = GROUND;
    MAP[0][1] = GROUND;
    MAP[0][2] = ROCK;
    MAP[0][3] = GROUND;
    MAP[0][4] = GROUND;
    MAP[0][5] = GROUND;

    MAP[1][0] = GROUND;
    MAP[1][1] = PIT;
    MAP[1][2] = GROUND;
    MAP[1][3] = GROUND;
    MAP[1][4] = SAND;
    MAP[1][5] = GROUND;

    MAP[2][0] = GROUND;
    MAP[2][1] = GROUND;
    MAP[2][2] = SAND;
    MAP[2][3] = GROUND;
    MAP[2][4] = GROUND;
    MAP[2][5] = PIT;

    MAP[3][0] = ROCK;
    MAP[3][1] = GROUND;
    MAP[3][2] = GROUND;
    MAP[3][3] = GROUND;
    MAP[3][4] = GROUND;
    MAP[3][5] = GROUND;

    MAP[4][0] = GROUND;
    MAP[4][1] = SAND;
    MAP[4][2] = GROUND;
    MAP[4][3] = GROUND;
    MAP[4][4] = ROCK;
    MAP[4][5] = GROUND;

    MAP[5][0] = GROUND;
    MAP[5][1] = GROUND;
    MAP[5][2] = GROUND;
    MAP[5][3] = PIT;
    MAP[5][4] = GROUND;
    MAP[5][5] = GROUND;

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            visited[i][j] = false;
        }
    }
}

void MapManagerLib::printMap() {
    for (int i = 0; i < size; i++) {
        Serial.print("| ");
        for (int j = 0; j < size; j++){
            Serial.print(MAP[j][i]);
            Serial.print(" ");
        }
        Serial.println("|");
    }
}

void MapManagerLib::printVisited() {
    for (int i = 0; i < size; i++) {
        Serial.print("| ");
        for (int j = 0; j < size; j++){
            Serial.print(visited[j][i]);
            Serial.print(" ");
        }
        Serial.println("|");
    }
}

void MapManagerLib::setMapValue(int x, int y, char value) {
    // assuming value is an acceptable char value
    // assuming that x and y are less than 6
    if (x < 0 || x > size - 1 || y < 0 || y > size - 1) {
        return;
    }
    if (MAP[x][y] != GROUND && MAP[x][y] != ITEM) {
        return;
    }
    MAP[x][y] = value;
}

void MapManagerLib::setVisited(int x, int y) {
    // assuming value is an acceptable char value
    // assuming that x and y are less than 6
    if (x < 0 || x > size - 1 || y < 0 || y > size - 1) {
        return;
    }
    visited[x][y] = true;
}

bool MapManagerLib::getVisited(int x, int y) {
    // assuming value is an acceptable char value
    // assuming that x and y are less than 6
    if (x < 0 || x > size - 1 || y < 0 || y > size - 1) {
        return false;
    }
    return visited[x][y];
}

char MapManagerLib::getMapValue(int x, int y) {
    // assuming that x and y values are less than size
    if (x < 0 || x > size - 1 || y < 0 || y > size - 1) {
        return OUTOFBOUNDS;
    }
    return MAP[x][y];
}

bool MapManagerLib::findClosest(int &x, int &y, int currentX, int currentY, char value) {
    float minDistance = 100;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (MAP[i][j] == value) {
                float distance = sqrt(pow(i - currentX, 2) + pow(j - currentY, 2));
                if (distance < minDistance) {
                    x = i;
                    y = j;
                    minDistance = distance;
                }
            }
        }
    }
    return minDistance == 100;
}

bool MapManagerLib::findClosestNotVisited(int &x, int &y, int currentX, int currentY, char value) {
    float minDistance = 100;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (MAP[i][j] == value && !visited[i][j]) {
                float distance = sqrt(pow(i - currentX, 2) + pow(j - currentY, 2));
                if (distance < minDistance) {
                    x = i;
                    y = j;
                    minDistance = distance;
                }
            }
        }
    }
    return minDistance == 100;
}

bool MapManagerLib::getLocation(int &x, int &y, char value) {
    // Returns true if value is found
    // x and y are assigned the coordinates of the value
    // This function would be used to find the goals (e.g person or candle)
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (MAP[i][j] == value) {
                x = i;
                y = j;
                return true;
            }
        }
    }
    return false;
}
