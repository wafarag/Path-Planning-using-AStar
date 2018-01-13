/*
 * A* Algorithm
 *
 * Defining the The World Map
 *
 * Created on: Jan 7, 2018
 *      Author: Wael Farag
 */

#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>

#define WHITE     7
#define RED       4
#define GREEN     2
#define BLUE      1
#define BLACK     15

struct Wall
{
  public:
    unsigned int x;
    unsigned int y;
};

// Define Map Walls
std::vector<Wall> MAP_Walls;

// In the MAP the coordinates in the upper left corner are (0,0) "the origin"

// Specify the Wall Cells in the Map (MAP #1)
void Set_Map1_Walls()
{
   Wall W;

   W = {1,0}; MAP_Walls.push_back(W);
   W = {1,1}; MAP_Walls.push_back(W);
   W = {1,2}; MAP_Walls.push_back(W);
   W = {1,3}; MAP_Walls.push_back(W);
   W = {1,4}; MAP_Walls.push_back(W);
   W = {1,5}; MAP_Walls.push_back(W);
   W = {1,6}; MAP_Walls.push_back(W);
   W = {1,7}; MAP_Walls.push_back(W);
   W = {1,8}; MAP_Walls.push_back(W);
   //W = {1,9}; MAP_Walls.push_back(W);

}

// Specify the Wall Cells in the Map (MAP #2)
void Set_Map2_Walls()
{
   Wall W;

   W = {3,0}; MAP_Walls.push_back(W);
   W = {3,1}; MAP_Walls.push_back(W);
   W = {3,2}; MAP_Walls.push_back(W);
   W = {3,3}; MAP_Walls.push_back(W);
   W = {3,4}; MAP_Walls.push_back(W);
   W = {3,5}; MAP_Walls.push_back(W);
   W = {3,6}; MAP_Walls.push_back(W);
   W = {3,7}; MAP_Walls.push_back(W);
   W = {3,8}; MAP_Walls.push_back(W);
   W = {3,9}; MAP_Walls.push_back(W);

   W = {6,2}; MAP_Walls.push_back(W);
   W = {6,3}; MAP_Walls.push_back(W);
   W = {6,4}; MAP_Walls.push_back(W);
   W = {6,5}; MAP_Walls.push_back(W);
   W = {6,6}; MAP_Walls.push_back(W);
   W = {6,7}; MAP_Walls.push_back(W);

   W = {10,4}; MAP_Walls.push_back(W);
   W = {10,5}; MAP_Walls.push_back(W);
   W = {10,6}; MAP_Walls.push_back(W);
   W = {10,7}; MAP_Walls.push_back(W);
   W = {10,8}; MAP_Walls.push_back(W);
   W = {10,9}; MAP_Walls.push_back(W);
   W = {10,10}; MAP_Walls.push_back(W);
   W = {10,11}; MAP_Walls.push_back(W);
   W = {10,12}; MAP_Walls.push_back(W);
   W = {10,13}; MAP_Walls.push_back(W);

   W = {15,0}; MAP_Walls.push_back(W);
   W = {15,1}; MAP_Walls.push_back(W);
   W = {15,2}; MAP_Walls.push_back(W);
   W = {15,3}; MAP_Walls.push_back(W);
   //W = {15,4}; MAP_Walls.push_back(W);
   W = {15,5}; MAP_Walls.push_back(W);
   W = {15,6}; MAP_Walls.push_back(W);
   W = {15,7}; MAP_Walls.push_back(W);
   W = {15,8}; MAP_Walls.push_back(W);
   W = {15,9}; MAP_Walls.push_back(W);
   W = {15,10}; MAP_Walls.push_back(W);
   W = {15,11}; MAP_Walls.push_back(W);
   W = {15,12}; MAP_Walls.push_back(W);
   W = {15,13}; MAP_Walls.push_back(W);
   W = {15,14}; MAP_Walls.push_back(W);
   W = {15,15}; MAP_Walls.push_back(W);
   W = {15,16}; MAP_Walls.push_back(W);
   W = {15,17}; MAP_Walls.push_back(W);
   W = {15,18}; MAP_Walls.push_back(W);
   W = {15,19}; MAP_Walls.push_back(W);
   W = {15,20}; MAP_Walls.push_back(W);
   W = {15,21}; MAP_Walls.push_back(W);
   W = {15,22}; MAP_Walls.push_back(W);
   W = {15,23}; MAP_Walls.push_back(W);
   W = {15,24}; MAP_Walls.push_back(W);
   W = {15,25}; MAP_Walls.push_back(W);
   W = {15,26}; MAP_Walls.push_back(W);
   W = {15,27}; MAP_Walls.push_back(W);
   W = {15,28}; MAP_Walls.push_back(W);
   W = {15,29}; MAP_Walls.push_back(W);

}

// Specify the Wall Cells in the Map (MAP #3)
void Set_Map3_Walls()
{
   Wall W;

   W = {8,0}; MAP_Walls.push_back(W);
   W = {10,0}; MAP_Walls.push_back(W);

   W = {0,1}; MAP_Walls.push_back(W);
   W = {1,1}; MAP_Walls.push_back(W);
   W = {2,1}; MAP_Walls.push_back(W);
   W = {3,1}; MAP_Walls.push_back(W);
   W = {4,1}; MAP_Walls.push_back(W);
   W = {5,1}; MAP_Walls.push_back(W);
   W = {6,1}; MAP_Walls.push_back(W);
   W = {8,1}; MAP_Walls.push_back(W);
   W = {10,1}; MAP_Walls.push_back(W);

   W = {8,2}; MAP_Walls.push_back(W);
   W = {10,2}; MAP_Walls.push_back(W);

   W = {8,3}; MAP_Walls.push_back(W);
   W = {10,3}; MAP_Walls.push_back(W);

   W = {8,4}; MAP_Walls.push_back(W);
   W = {10,4}; MAP_Walls.push_back(W);

   W = {8,5}; MAP_Walls.push_back(W);
   W = {10,5}; MAP_Walls.push_back(W);

   W = {4,6}; MAP_Walls.push_back(W);
   W = {5,6}; MAP_Walls.push_back(W);
   W = {6,6}; MAP_Walls.push_back(W);
   W = {7,6}; MAP_Walls.push_back(W);
   W = {8,6}; MAP_Walls.push_back(W);
   W = {10,6}; MAP_Walls.push_back(W);

   W = {10,7}; MAP_Walls.push_back(W);

   W = {1,8}; MAP_Walls.push_back(W);
   W = {2,8}; MAP_Walls.push_back(W);
   W = {3,8}; MAP_Walls.push_back(W);
   W = {4,8}; MAP_Walls.push_back(W);
   W = {10,8}; MAP_Walls.push_back(W);

   W = {1,9}; MAP_Walls.push_back(W);
   W = {4,9}; MAP_Walls.push_back(W);
   W = {10,9}; MAP_Walls.push_back(W);

   W = {1,10}; MAP_Walls.push_back(W);
   W = {4,10}; MAP_Walls.push_back(W);
   W = {10,10}; MAP_Walls.push_back(W);

   W = {1,11}; MAP_Walls.push_back(W);
   W = {2,11}; MAP_Walls.push_back(W);
   W = {3,11}; MAP_Walls.push_back(W);
   W = {4,11}; MAP_Walls.push_back(W);
   W = {8,11}; MAP_Walls.push_back(W);
   W = {9,11}; MAP_Walls.push_back(W);
   W = {10,11}; MAP_Walls.push_back(W);

   W = {8,12}; MAP_Walls.push_back(W);
   W = {10,12}; MAP_Walls.push_back(W);

   W = {10,13}; MAP_Walls.push_back(W);

}

void Construct_MAP()
{
    // MAP #1 (WORLD Size)
    //#define N_COLS   4    // x
    //#define N_RAWS   10   // y
    //Set_Map1_Walls();

    // MAP #2 (WORLD Size)
    //#define N_COLS   20   // x
    //#define N_RAWS   30   // y
    //Set_Map2_Walls();

    // MAP #2 (WORLD Size)
    #define N_COLS   11   // x
    #define N_RAWS   14   // y
    Set_Map3_Walls();


}


// Check if the given position is position of a Wall or a clear cell.

bool Is_Cell_Clear(unsigned int x_pos, unsigned int y_pos)
{
    bool State = true;

    for (unsigned int i =0; i < MAP_Walls.size(); i++)
    {
        if ( (x_pos == MAP_Walls[i].x) and (y_pos == MAP_Walls[i].y))
        {
            State = false;
        }
    }

    return State;
}


#endif /* MAP_H */
