#ifndef MapManagerLib_h
#define MapManagerLib_h

#include "Arduino.h"

class MapManagerLib
{
    private:
        char MAP[6][6];
        const int size = 6;

    public:
        MapManagerLib();
        
        // Const values
        const char UNKNOWN = '?';
        const char WATER = 'W';
        const char GROUND = 'G';
        const char PROPABLY_GROUND = 'PG';
        const char PERSON = 'P';
        const char GROUP = 'GP';
        const char FINISH = 'F';
        const char SAND = 'S';
        const char ROCK = 'R';
        const char CANDLE = 'C';
        const char ITEM = 'I';

        void setMapValue(int x, int y, char value);
        char getMapValue(int x, int y);
        bool getLocation(int &x, int &y, char value);
};

#endif