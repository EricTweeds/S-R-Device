#ifndef MapManagerLib_h
#define MapManagerLib_h

#include "Arduino.h"

class MapManagerLib
{
    public:
        MapManagerLib();
        
        // Const values
        const char PIT = 'W';
        const char GROUND = 'G';
        const char PERSON = 'P';
        const char GROUP = 'A';
        const char FINISH = 'F';
        const char SAND = 'S';
        const char ROCK = 'R';
        const char CANDLE = 'C';
        const char ITEM = 'I';

        void setMapValue(int x, int y, char value);
        char getMapValue(int x, int y);
        bool getLocation(int &x, int &y, char value);
        void printMap();

    private:
        char MAP[6][6];
        // char MAP[6][6] = {
        //     {GROUND, GROUND, GROUND, ROCK, GROUND, GROUND},
        //     {GROUND, PIT, GROUND, GROUND, SAND, GROUND},
        //     {ROCK, GROUND, SAND, GROUND, GROUND, GROUND},
        //     {GROUND,GROUND,GROUND,GROUND,GROUND,PIT},
        //     {GROUND, SAND, GROUND, GROUND, ROCK, GROUND},
        //     {GROUND, GROUND, PIT, GROUND, GROUND, GROUND}
        // };
        const int size = 6;
};

#endif