/*
 * A* Algorithm
 *
 * Defining Path Finding functions
 *
 * Created on: Jan 7, 2018
 *      Author: Wael Farag
 */

#include "Cell.hpp"
#include <cmath>


// The Grid is our Search Space - 2D array
vector<vector<Cell*> > Grid(N_COLS, vector<Cell*>(N_RAWS)); // 2D array

// Define Open and Closed Lists, as well as the Solution Path List
vector<Cell*> openList;
vector<Cell*> closedList;
vector<Cell*> pathList;

// Define the positions of Start and End Points
Cell *StartPos;
Cell *EndPos;
// Define a container for the current Robot Position
Cell *CurrentPos;

// Function to delete an element from any List (Open, Closed ... etc)
vector<Cell*> removeFromList(vector<Cell*> List, Cell* Element_Cell)
{
    for (unsigned int i = 0; i <= List.size(); i++)
        {
            if ((List[i]->x == Element_Cell->x) and (List[i]->y == Element_Cell->y))
                {
                    List.erase(List.begin() + i );
                }
        }
    return List;
}


// Function to Calculate the Heuristic Value "H" for a Cell - based on Euclidean Distance
float Euclidean_Dist_Heuristic(Cell* First_Cell, Cell* Second_Cell)
{
    float dist = (float) sqrt((First_Cell->x - Second_Cell->x)*(First_Cell->x - Second_Cell->x) +
                 (First_Cell->y - Second_Cell->y)*(First_Cell->y - Second_Cell->y));

    return dist;
}

// Function to Calculate the Heuristic Value "H" for a Cell - based on Manhattan Distance
float Manhattan_Dist_Heuristic(Cell* First_Cell, Cell* Second_Cell)
{
    float dist = (float)(fabs(First_Cell->x - Second_Cell->x) + fabs(First_Cell->y - Second_Cell->y));

    return dist;
}

// Function to check if an Element is included in a Vector (List) or not
bool Includes(vector<Cell*> List, Cell* Element_Cell)
{
    bool flag = false;
    for (unsigned int i = 0; i < List.size(); i++)
        {
            if ((List[i]->x == Element_Cell->x) and (List[i]->y == Element_Cell->y))
                {
                    flag = true;
                }
        }
    return flag;
}

// Set up the search Grid
void Initialize_Grid()
{
    // Initialize the Grid with Cell Coordinates
    for (unsigned int i = 0; i < N_COLS; i++)
    {
        for (unsigned int j = 0; j < N_RAWS; j++)
            {
                Cell *C = new Cell(i,j);
                Grid[i][j] = C;
            }
    }
    // Find  all the neighbors for all Grid Cells
    for (unsigned int i = 0; i < N_COLS; i++)
    {
        for (unsigned int j = 0; j < N_RAWS; j++)
            {
                Grid[i][j]->Find_Neighbors(Grid);
            }
    }
}

// The Core of the A* Algorithm
void Search_Path()
{
    bool Solution_Found = false;

    // Loop till all the cells in the openList are checked
    while (openList.size() > 0)
    {
        unsigned int index_Lowest_F = 0;

        for (unsigned int i = 0; i < openList.size(); i++)
        {
            if (openList[i]->F() < openList[index_Lowest_F]->F())
            {
                index_Lowest_F = i;
            }
        }
        CurrentPos = openList[index_Lowest_F]; // move the Robot to this position with lowest F

        // Check if the Robot is already reached the final position
        if ((CurrentPos->x == EndPos->x) and (CurrentPos->y == EndPos->y))
        {
            Solution_Found = true;
            break;
        }

        // Best position (Robot) moves from openList to closedList
        openList = removeFromList(openList, CurrentPos);
        closedList.push_back(CurrentPos);

        // check all the neighboring cells
        CurrentPos->Find_Neighbors(Grid);
        vector<Cell*> neighbors = CurrentPos->neighbors;

        for (unsigned int i = 0; i < neighbors.size(); i++)
        {
          Cell* neighbor = neighbors[i];

          // Does the next Cell is a valid one?
          if (!Includes(closedList, neighbor) and !neighbor->Is_Wall)
          {
            float newG = CurrentPos->G + Euclidean_Dist_Heuristic(neighbor, CurrentPos);

            // Is this a better path than before?
            bool newPath_found = false;

            if (Includes(openList, neighbor))
            {
                if (newG < neighbor->G)
                {
                    neighbor->G = newG;
                    newPath_found = true;
                }
            } else
            {
                neighbor->G = newG;
                newPath_found = true;
                openList.push_back(neighbor);
            }

            // Yes, it is a better path
            if (newPath_found)
            {
               //neighbor->H = Euclidean_Dist_Heuristic(neighbor, EndPos);
               neighbor->H = Manhattan_Dist_Heuristic(neighbor, EndPos);
               neighbor->PreviousPos = CurrentPos;
            }
          }
        }
    }

    if (Solution_Found)
    {
        cout << "Cheers, A Solution is Found !!" << endl;
        cout << "Current Position x =" << CurrentPos->x << endl;
        cout << "Current Position y =" << CurrentPos->y << endl;

    } else
    {
       cout << "Sorry, No Solution is found ..." << endl;
       cout << "Current Position x =" << CurrentPos->x << endl;
       cout << "Current Position y =" << CurrentPos->y << endl;
    }
}

void Display_Grid()
{
    SetConsoleTextAttribute(console, WHITE);
    cout << endl << "-------------- Displaying the Grid -------------" << endl << endl;

    for (unsigned int i = 0; i < N_COLS; i++)
    {
        for (unsigned int j = 0; j < N_RAWS; j++)
        {
            Grid[i][j]->Draw(WHITE);
        }
    }
}

void Display_OpenList()
{
    SetConsoleTextAttribute(console, BLUE);
    cout << endl << "-------------- Displaying the Open Cell List -------------" << endl << endl;

    for (unsigned int i = 0; i < openList.size(); i++)
    {
        openList[i]->Draw(BLUE);
    }
}

void Display_ClosedList()
{
    SetConsoleTextAttribute(console, RED);
    cout << endl << "-------------- Displaying the Closed Cell List -------------" <<endl<<endl;

    for (unsigned int i = 0; i < closedList.size(); i++)
    {
        closedList[i]->Draw(RED);
    }
    cout << endl<< "Number of Searched Cells: " << closedList.size() << endl<<endl;
}

void Display_Path()
{
    SetConsoleTextAttribute(console, GREEN);
    cout << endl << "-------------- Displaying the Path Cell List -------------" <<endl<<endl;

    // Find the list of the path cells by working backwards
    Cell* pathCell = CurrentPos;
    pathList.push_back(pathCell);
    float pathCost = pathCell->G;

    while (pathCell->PreviousPos != NULL)
    {
        pathList.push_back(pathCell->PreviousPos);
        pathCell = pathCell->PreviousPos;
    }

    for (unsigned int i = 0; i < pathList.size(); i++)
    {
        pathList[i]->Draw(GREEN);
    }

    cout<<endl<<" Total Path Cost =" << pathCost << endl << endl;
}


void Draw_Final_Grid()
{
    SetConsoleTextAttribute(console, WHITE);
    cout << endl << "------------------------ Drawing the Final Grid -----------------------" <<endl;
    cout << endl << "Legend: RED -> 'Wall', GREEN -> 'Path Cell', WHITE -> 'Normal Grid Cell'. " <<endl<<endl;

    for (unsigned int j = 0; j < N_RAWS; j++)
    {
        cout << string(20, ' ');

        for (unsigned int i = 0; i < N_COLS; i++)
        {
            if(!Is_Cell_Clear(Grid[i][j]->x, Grid[i][j]->y))
            {
                SetConsoleTextAttribute(console, RED);
                cout << "*";
            }
            else
            {
                if(Includes(pathList, Grid[i][j]))
                {
                    SetConsoleTextAttribute(console, GREEN);
                    cout << "*";
                }
                else
                {
                    SetConsoleTextAttribute(console, WHITE);
                    cout << "*";
                }
            }
        }
        cout<<endl;
    }
}












