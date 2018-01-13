/*
 * A* Algorithm
 *
 * Main Pipeline
 *
 * Created on: Jan 7, 2018
 *      Author: Wael Farag
 */

#include <stdlib.h>
#include "Path_Finding.hpp"

// Define Start & End Positions Coordinates
#define StartPOS_x  1
#define StartPOS_y  0
#define EndPOS_x    9
#define EndPOS_y    13

int main()
{
    // Setup the MAP "The World"
    Construct_MAP();
    // Setup the Map Grid to start search
    Initialize_Grid();

    // Define the Start and End Positions for the Robot
    if ((StartPOS_x > N_COLS) or (StartPOS_y > N_RAWS)) { cout << " Error in Start or End Positions" <<endl; exit(0);}
    if ((EndPOS_x > N_COLS)   or (EndPOS_y > N_RAWS)  ) { cout << " Error in Start or End Positions" <<endl; exit(0);}
    StartPos = Grid[StartPOS_x][StartPOS_y];
    EndPos   = Grid[EndPOS_x][EndPOS_y];

    // openList starts with the beginning only
    openList.push_back(StartPos);
    // Start Searching
    Search_Path();

    // Display the current status
    // You comment or uncomment the lines down there base on your needs
    //Display_Grid();
    Display_ClosedList();
    Display_OpenList();
    Display_Path();
    Draw_Final_Grid();

    return 0;
}
