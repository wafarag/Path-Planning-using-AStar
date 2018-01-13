/*
 * A* Algorithm
 *
 * Defining the "Cell" class
 *
 * Created on: Jan 7, 2018
 *      Author: Wael Farag
 */

#ifndef CELL_H
#define CELL_H

#include "MAP.hpp"
#include <string>
#include <conio.h>
#include <windows.h>

using namespace std;

HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

class Cell
{
  public:
    // Position
    unsigned int x;
    unsigned int y;

    // G, H values for the cell
    float G;     // Cost
    float H;     // Heuristic

    bool Is_Wall;  // Flag to determine if this Cell is "Clear" or a "Wall"

    // Neighboring cells: range from 3 to 8 neighbors
    vector<Cell*> neighbors;

    // Where the Robot was in the previous step (Previous Position)
    Cell *PreviousPos;

    // Constructor: Assign Cell coordinates
    Cell(unsigned int x, unsigned int y)
    {
        this->x = x;
        this->y = y;

        this->G = 0.0;
        this->H = 0.0;
        Is_Wall = Is_a_Wall(this->x, this->y);
        this->PreviousPos = NULL;
    };

    // Destructor
    virtual ~Cell() {};

    // Check if the specified cell is a Wall or not
    bool Is_a_Wall(unsigned int x_pos, unsigned int y_pos)
    {
        bool   flag = !Is_Cell_Clear(x_pos, y_pos);
        return flag;
    }

    // Calculate F
    float F()
    {
        return (this->G + this->H);
    }

    // Draw Cell - We just write the cell coordinated Now.
    // In the future it should Draw the cell using graphics
    void Draw(unsigned int COLOR)
    {
        SetConsoleTextAttribute(console, COLOR);

        cout << " Cell ( " << this->x <<", "<<this->y <<")"<< endl;
    }

    // Assign Neighbors for each cell in the Grid
    void Find_Neighbors(vector<vector<Cell*> > Grid)
    {
        // Expected at most 8 Neighboring Cells and minimum 3

        // Find the one direct to the right
        if (this->x < (N_COLS - 1)) neighbors.push_back(Grid[this->x + 1][this->y]);
        // Find the one direct to the left
        if (this->x > 0) neighbors.push_back(Grid[this->x - 1][this->y]);
        // Find the one direct below
        if (this->y < (N_RAWS - 1)) neighbors.push_back(Grid[this->x][this->y + 1]);
        // Find the one direct up
        if (this->y > 0) neighbors.push_back(Grid[this->x][this->y - 1]);
        // Find the one diagonally up left
        if (this->x > 0 and this->y > 0) neighbors.push_back(Grid[this->x -1][this->y - 1]);
        // Find the one diagonally up right
        if (this->x < (N_COLS - 1) and this->y > 0) neighbors.push_back(Grid[this->x + 1][this->y - 1]);
        // Find the one diagonally down left
        if (this->x > 0 and this->y < (N_RAWS - 1)) neighbors.push_back(Grid[this->x - 1][this->y + 1]);
        // Find the one diagonally down right
        if (this->x < (N_COLS - 1) and this->y < (N_RAWS - 1)) neighbors.push_back(Grid[this->x + 1][this->y + 1]);

    }

};


#endif /* CELL_H */
