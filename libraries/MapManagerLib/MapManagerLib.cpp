#include "Arduino.h"
#include "MapManagerLib.h"

MapManagerLib::MapManagerLib() {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++){
            MAP[i][j] = UNKNOWN;
        }
    }
}

void printMap() {
    for (int i = 0; i < size; i++) {
        Serial.print("| ")
        for (int j = 0; j < size; j++){
            Serial.print(MAP[i][j]);
            Serial.print(" ");
        }
        Serial.println("|");
    }
}

void MapManagerLib::setMapValue(int x, int y, char value) {
    // assuming value is an acceptable char value
    // assuming that x and y are less than 6
    MAP[x][y] = value;
}

char MapManagerLib::getMapValue(int x, int y) {
    // assuming that x and y values are less than size
    return MAP[x][y];
}

bool MapManagerLib::getLocation(int &x, int &y, char value) {
    // Returns true if value is found
    // x and y are assigned the coordinates of the value
    // This function would be used to find the goals (e.g person or candle)
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++){
            if (MAP[i][j] == value) {
                x = i;
                j = y;
                return true;
            }
        }
    }
    return false;
}
