 /*-------------------------------------------------------------------------+
 |	Final Project CSC412 - Spring 2023										|
 |	A graphic front end for a box-pushing simulation.						|
 |																			|
 |	This application simply creates a glut window with a pane to display	|
 |	a colored grid and the other to display some state information.			|
 |																			|
 |	Current GUI:															|
 |		- 'ESC' --> exit the application									|
 |		- ',' --> slows down the simulation									|
 |		- '.' --> apeeds up  he simulation									|
 |																			|
 |	Created by Jean-Yves Hervé on 2018-12-05 (C version)					|
 |	Revised 2023-04-27														|
 +-------------------------------------------------------------------------*/
//
//  main.cpp
//  Final Project CSC412 - Spring 2023
//
//  Created by Jean-Yves Hervé on 2018-12-05, Rev. 2023-12-01
//	This is public domain code.  By all means appropriate it and change is to your
//	heart's content.
#include <iostream>
#include <sstream>
#include <string>
#include <random>
#include <vector>
#include <map>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <fstream>
#include <thread>
#include <chrono>
//
//
#include "glPlatform.h"
#include "typesAndConstants.h"
#include "gl_frontEnd.h"

using namespace std;

#if 0
//=================================================================
#pragma mark -
#pragma mark Function prototypes
//=================================================================
#endif

void displayGridPane(void);
void displayStatePane(void);
void printHorizontalBorder(ostringstream& outStream);
string printGrid(void);
void initializeApplication(void);
void cleanupAndQuit();
void generatePartitions();
void* robotFunc(int thisIndex);

#if 0
//=================================================================
#pragma mark -
#pragma mark Application-level global variables
//=================================================================
#endif

//	Don't touch
extern int	gMainWindow, gSubwindow[2];

//-------------------------------------
//	Don't rename any of these variables
//-------------------------------------
//	The state grid's dimensions (arguments to the program)
int numRows = -1;	//	height of the grid
int numCols = -1;	//	width
int numBoxes = -1;	//	also the number of robots
int numDoors = -1;	//	The number of doors.

int numLiveThreads = 0;		//	the number of live robot threads

//	robot sleep time between moves (in microseconds)
int robotSleepTime = 100000;

//	An array of C-string where you can store things you want displayed
//	in the state pane to display (for debugging purposes?)
//	Dont change the dimensions as this may break the front end
const int MAX_NUM_MESSAGES = 8;
const int MAX_LENGTH_MESSAGE = 32;
char** message;

//	Only absolutely needed if you tackle the partition EC
SquareType** grid;

//
ofstream outFile("robotSimulOut.txt");

//-----------------------------
//	CHANGE THIS
//-----------------------------
//	Here I hard-code myself some data for robots and doors.  Obviously this code
//	must go away.  I just want to show you how information gets displayed.  
//	Obviously, you will need to allocate yourself some dynamic data structure to store 
//	that information once you know the dimensions of the grid, number of boxes/robots and
//	doors.  
//	Note that, even if you use the GUI version, it doesn't impose you a storage format at
//	all, since the drawing function draw a single robot/box/door at a time, and take that
//	object's parameters as individual argumenta.
//	So, feel free to go vectors all the way if you like it better than int**
//	Just don't feel free to declare oversized arrays, in the style of
//	int robotLoc[1000][2];
//	I can guarantee you that this kind of stuff will get penalized harshly (if might have
//	been convenient, borderline cute, in CSC211, but by now it's absolutely embarrassing)
//
//	Also:  Please note that because this is a grid-based problem, I never think of x and y but
//			row and column (well, the "I" dealing with the planning problem.  The "I" doing
//			the dirty work underneath has to translate all of that row and column data into
//			x and y pixel coordinates for OpenGL for rendering
//		   So,... In all of these arrays of 2 int values, the first value (index 0)
//			is a row coordinate and the second value (index 1) is a column coordinate.
// int doorAssign[] = {1, 0, 0, 2, 1, 3};	//	door id assigned to each robot-box pair
// int robotLoc[][2] = {{12, 8}, {6, 9}, {3, 14}, {11, 15}, {14, 1}, {8, 13}};
// int boxLoc[][2] = {{6, 7}, {4, 12}, {13, 13}, {8, 12}, {7, 14}, {11, 9}};
// int doorLoc[][2] = {{3, 3}, {8, 11}, {7, 10}, {12, 6}};

//	The above hard-coded intialization should be replaced by random generation in
//	initializeApplication().
//	Of course, this means that you need to modify the type of the above variables
//int** robotLoc;
//int** boxLoc;
//int** doorAssign;
//int** doorLoc;
//	Or with a bit of retooling
std::vector<int> doorAssign;
std::vector<GridPosition> robotLoc;
std::vector<GridPosition> boxLoc;
std::vector<GridPosition> doorLoc;

std::vector<std::thread> robotThreadVec;
std::vector<bool> RobotLiveVec;
std::mutex fileMutex;

//	For extra credit section
random_device randDev;
default_random_engine engine(randDev());
vector<SlidingPartition> partitionList;
//	Change argument to 0.5 for equal probability of vertical and horizontal partitions
//	0 for only horizontal, and 1 for only vertical
bernoulli_distribution headsOrTails(1.0);

#if 0
//=================================================================
#pragma mark -
#pragma mark Function implementations
//=================================================================
#endif

void validateInput(int argc, char** argv) {
	if (argc != 5) {
		printf("Incorrect number of command line arguments. Please enter './<exeName> <numRows> <numCols> <numBoxes> <numDoors'\n");
		exit(1);
	}

	numRows = atoi(argv[1]);
	if (numRows <= 0) {
		printf("The number of rows must be a strictly positive integer\n");
		exit(1);
	}

	numCols = atoi(argv[2]);
	if (numCols <= 0) {
		printf("The number of columns must be a strictly positive integer\n");
		exit(1);
	}

	numLiveThreads = atoi(argv[3]);
	if (numLiveThreads <= 0) {
		printf("The number of boxes must be a strictly positive integer\n");
		exit(1);
	}

	numDoors = atoi(argv[4]);
	if (numDoors <= 0 || numDoors >= 4) {
		printf("The number of doors must be a strictly positive integer between 1 and 3\n");
		exit(1);
	}
	
	if (((numLiveThreads * 2) + numDoors) > (numRows * numCols)) {
		printf("The grid size is not large enough to run the simulation with the input door count and input box count\n");
		exit(1);
	}
	return;
}

int main(int argc, char** argv)
{
	//	We know that the arguments  of the program  are going
	//	to be the width (number of columns) and height (number of rows) of the
	//	grid, the number of boxes (and robots), and the number of doors.
	//	You are going to have to extract these.  For the time being,
	//	I hard code-some values
	numRows = 16;
	numCols = 20;
	numDoors = 3;
	numBoxes = 3;
	// validateInputs(argc, argv);
	outFile << numRows << " " << numCols << " " << numDoors << " " << numBoxes << "\n\n";

	//	Even though we extracted the relevant information from the argument
	//	list, I still need to pass argc and argv to the front-end init
	//	function because that function passes them to glutInit, the required call
	//	to the initialization of the glut library.
	initializeFrontEnd(argc, argv, displayGridPane, displayStatePane);

	//	Now we can do application-level initialization
	initializeApplication();

	string outStr = printGrid();
	cout << outStr << endl;
	
	//	Now we enter the main loop of the program and to a large extend
	//	"lose control" over its execution.  The callback functions that 
	//	we set up earlier will be called when the corresponding event
	//	occurs

	glutMainLoop();
	
	cleanupAndQuit();
		
	//	This will probably never be executed (the exit point will be in one of the
	//	call back functions).
	return 0;
}

void cleanupAndQuit()
{
	// Stop all robots
	for (int i = 0; i < numBoxes; i++) {
		RobotLiveVec[i] = false;
	}

	// Wait to ensure that all robots are finished
	std::this_thread::sleep_for(std::chrono::microseconds(robotSleepTime));

	// Join the robot threads
	for (int i = 0; i < numBoxes; i++) {
		robotThreadVec[i].join();
	}

	// Close the output file
	outFile.close();

	// Unallocate any allocated data
	for (int i=0; i< numRows; i++)
		delete []grid[i];
	delete []grid;
	for (int k=0; k<MAX_NUM_MESSAGES; k++)
		delete []message[k];
	delete []message;

	exit(0);
}

// --------------------------------------------------------------------------------------------------------------

// This function generates a random row and random column including the edges of the grid
void getNewRowColEdge(unsigned int &row, unsigned int &col, std::vector<GridPosition>* usedCoords) {
	static std::uniform_int_distribution<unsigned int> randEdgeRow(0, numRows - 1);
	static std::uniform_int_distribution<unsigned int> randEdgeCol(0, numCols - 1);

	bool isDupeCoords = true;
	while (isDupeCoords) {
		isDupeCoords = false;
		row = randEdgeRow(engine);
		col = randEdgeCol(engine);
		for (long unsigned int j = 0; j < usedCoords->size(); j++) {
			if ((row == (*usedCoords)[j].row) && (col == (*usedCoords)[j].col)) {
				isDupeCoords = true;
				break;
			}
		}
	}
	return;
}

// This function generates a random row and random column excluding the edges of the grid
void getNewRowColnonEdge(unsigned int &row, unsigned int &col, std::vector<GridPosition>* usedCoords) {
	static std::uniform_int_distribution<unsigned int> randRow(1, numRows - 2);
	static std::uniform_int_distribution<unsigned int> randCol(1, numCols - 2);

	bool isDupeCoords = true;
	while (isDupeCoords) {
		isDupeCoords = false;
		row = randRow(engine);
		col = randCol(engine);
		for (long unsigned int j = 0; j < usedCoords->size(); j++) {
			if ((row == (*usedCoords)[j].row) && (col == (*usedCoords)[j].col)) {
				isDupeCoords = true;
				break;
			}
		}
	}
	return;
}

// Function which writes out starting the basic starting information to the output
// file
void writeStartingPositions() {
	for (int i = 0; i < numDoors; i++) {
		if (i == numDoors - 1) {
			outFile << "(" << doorLoc[i].row << ", " << doorLoc[i].col << ")\n\n";
			break;
		}
		outFile << "(" << doorLoc[i].row << ", " << doorLoc[i].col << ") ";
	}

	for (int i = 0; i < numBoxes; i++) {
		if (i == numBoxes - 1) {
			outFile << "(" << boxLoc[i].row << ", " << boxLoc[i].col << ")\n\n";
			break;
		}
		outFile << "(" << boxLoc[i].row << ", " << boxLoc[i].col << ") ";
	}

	for (int i = 0; i < numBoxes; i++) {
		if (i == numBoxes - 1) {
			outFile << "(" << robotLoc[i].row << ", " << robotLoc[i].col << ") Door: " << doorAssign[i] << "\n\n";
			break;
		}
		outFile << "(" << robotLoc[i].row << ", " << robotLoc[i].col << ") Door: " << doorAssign[i] << " ";
	}
}

void initializeApplication(void)
{
	//	Allocate the grid
	grid = new SquareType*[numRows];
	for (int i=0; i<numRows; i++)
		grid[i] = new SquareType [numCols];
	
	message = new char*[MAX_NUM_MESSAGES];
	for (int k=0; k<MAX_NUM_MESSAGES; k++)
		message[k] = new char[MAX_LENGTH_MESSAGE+1];


	static std::uniform_int_distribution<int> randDoor(0, numDoors - 1);
	std::vector<GridPosition> usedCoords;
	
	// Generate Robot and Boxes
	for (int i = 0; i < numBoxes; i++) {
		unsigned int thisRow = 0;
		unsigned int thisCol = 0;

		// Create RobotStart
		getNewRowColEdge(thisRow, thisCol, &usedCoords);
		GridPosition robotStart = {thisRow, thisCol};
		usedCoords.push_back(robotStart);
		robotLoc.push_back(robotStart);
		RobotLiveVec.push_back(true);

		// Create BoxStart
		getNewRowColnonEdge(thisRow, thisCol, &usedCoords);
		GridPosition boxStart = {thisRow, thisCol};
		usedCoords.push_back(boxStart);
		boxLoc.push_back(boxStart);

		// Assign Door Number
		doorAssign.push_back(randDoor(engine));
	}

	// Generate Doors
	for (int i = 0; i < numDoors; i++) {
		unsigned int thisRow = 0;
		unsigned int thisCol = 0;

		getNewRowColEdge(thisRow, thisCol, &usedCoords);
		GridPosition thisDoor = {thisRow, thisCol};
		usedCoords.push_back(thisDoor);
		doorLoc.push_back(thisDoor);
	}

	// Write first 3 lines to the file
	writeStartingPositions();
	
	// Start the Robot Threads
	for (int k = 0; k < numBoxes; k++) {
		robotThreadVec.push_back(std::thread (robotFunc, k));
	}
}

// Function which moves the robot
void move(char direction, int index) {
	switch (direction) {
		case 'N':
			robotLoc[index].row -= 1;
			break;
		case 'E':
			robotLoc[index].col += 1;
			break;
		case 'S':
			robotLoc[index].row += 1;
			break;
		case 'W':
			robotLoc[index].col -= 1;
			break;
		default:
			break;
	}
}

// Function which moves the robot and its box
void push(char direction, int index) {
	switch (direction) {
		case 'N':
			robotLoc[index].row -= 1;
			boxLoc[index].row -= 1;
			break;
		case 'E':
			robotLoc[index].col += 1;
			boxLoc[index].col += 1;
			break;
		case 'S':
			robotLoc[index].row += 1;
			boxLoc[index].row += 1;
			break;
		case 'W':
			robotLoc[index].col -= 1;
			boxLoc[index].col -= 1;
			break;
		default:
			break;
	}
}

// This function puts the instructions into the directions and instructions vectors which involve the
// robot moving to the box in the horizontal direction
void HorizMoveToBox(std::vector<char>* directions, std::vector<char>* instructions, int idealRowDist, int idealColDist, int boxColDist) {
	int robotToStartCol = idealColDist;
	
	if ((idealRowDist == 0) && ((abs(idealColDist)) > (abs(boxColDist)))) {
		(*directions).push_back('S');
		(*instructions).push_back('M');
	}

	while (robotToStartCol != 0) {
		if (robotToStartCol > 0) {
			(*directions).push_back('E');
			robotToStartCol -= 1;
		} else {
			(*directions).push_back('W');
			robotToStartCol += 1;
		}
		(*instructions).push_back('M');
	}

	if ((idealRowDist == 0) && ((abs(idealColDist)) > (abs(boxColDist)))) {
		(*directions).push_back('N');
		(*instructions).push_back('M');
	}
	return;
}

// This function puts the instructions into the directions and instructions vectors which involve the
// robot moving to the box in the vertical direction
void VertMoveToBox (std::vector<char>* directions, std::vector<char>* instructions, int idealRowDist, int boxRowDist) {
	int robotToStartRow = idealRowDist;

	if ((abs(idealRowDist)) > (abs(boxRowDist))) {
		(*directions).push_back('W');
		(*instructions).push_back('M');
	}

	while (robotToStartRow != 0) {
		if (robotToStartRow > 0) {
			(*directions).push_back('S');
			robotToStartRow -= 1;
		} else {
			(*directions).push_back('N');
			robotToStartRow += 1;
		}
		(*instructions).push_back('M');
	}

	if ((abs(idealRowDist)) > (abs(boxRowDist))) {
		(*directions).push_back('E');
		(*instructions).push_back('M');
	}
	return;
}

// This function puts the instructions into the directions and instructions vectors which involve the
// robot pushing the box to the door in the horizontal direction
void HorizPushToDoor (std::vector<char>* directions, std::vector<char>* instructions, int boxToDoorCol) {
	while (boxToDoorCol != 0) {
		if (boxToDoorCol > 0) {
			(*directions).push_back('E');
			boxToDoorCol -= 1;
		} else {
			(*directions).push_back('W');
			boxToDoorCol += 1;
		}
		(*instructions).push_back('P');
	}
	return;
}

// This function puts the instructions into the directions and instructions vectors which involve the
// reposition the robot so that it can push the box to the door in the vertical direction
void PositionReadjusted (std::vector<char>* directions, std::vector<char>* instructions, int boxToDoorRow) {
	if ((*instructions)[(*instructions).size() - 1] == 'P') {
		if (boxToDoorRow < 0) {
			if ((*directions)[(*directions).size()-1] == 'E') {
				(*directions).push_back('S');
				(*directions).push_back('E');
				(*instructions).push_back('M');
				(*instructions).push_back('M');
			} else {
				(*directions).push_back('S');
				(*directions).push_back('W');
				(*instructions).push_back('M');
				(*instructions).push_back('M');
			}
		} else if (boxToDoorRow > 0) {
			if ((*directions)[(*directions).size()-1] == 'E') {
				(*directions).push_back('N');
				(*directions).push_back('E');
				(*instructions).push_back('M');
				(*instructions).push_back('M');
			} else {
				(*directions).push_back('N');
				(*directions).push_back('W');
				(*instructions).push_back('M');
				(*instructions).push_back('M');
			}
		}
	}
	return;
}

// This function puts the instructions into the directions and instructions vectors which involve the
// reposition the robot so that it can push the box to the door in the vertical direction
void VertPushToBox (std::vector<char>* directions, std::vector<char>* instructions, int boxToDoorRow) {
	while (boxToDoorRow != 0) {
		if (boxToDoorRow > 0) {
			(*directions).push_back('S');
			boxToDoorRow -= 1;
		} else {
			(*directions).push_back('N');
			boxToDoorRow += 1;
		}
		(*instructions).push_back('P');
	}
	return;
}

// Function to run each thread on
void* robotFunc(int thisIndex)
{	
	// Getting directions from the box to the door
	int boxToDoorRow = doorLoc[doorAssign[thisIndex]].row - boxLoc[thisIndex].row;
	int boxToDoorCol = doorLoc[doorAssign[thisIndex]].col - boxLoc[thisIndex].col;
	
	// Getting the Position of the ideal spot to move the robot
	GridPosition idealRobotStart;
	if (boxToDoorCol > 0) {
		idealRobotStart = {boxLoc[thisIndex].row, boxLoc[thisIndex].col - 1};
	} else if (boxToDoorCol < 0) {
		idealRobotStart = {boxLoc[thisIndex].row, boxLoc[thisIndex].col + 1};
	} else {
		if (boxToDoorRow > 0) {
			idealRobotStart = {boxLoc[thisIndex].row - 1, boxLoc[thisIndex].col};
		} else {
			idealRobotStart = {boxLoc[thisIndex].row + 1, boxLoc[thisIndex].col};
		}
	}

	std::vector<char> directions;
	std::vector<char> instructions;

	// Getting directions from the robot to the ideal start point
	int idealRowDist = idealRobotStart.row - robotLoc[thisIndex].row;
	int idealColDist = idealRobotStart.col - robotLoc[thisIndex].col;
	int boxRowDist = boxLoc[thisIndex].row - robotLoc[thisIndex].row;
	int boxColDist = boxLoc[thisIndex].col - robotLoc[thisIndex].col;

	// Pushing Instructions and Directions into directions and instructions vectors
	HorizMoveToBox(&directions, &instructions, idealRowDist, idealColDist, boxColDist);
	VertMoveToBox(&directions, &instructions, idealRowDist, boxRowDist);
	HorizPushToDoor(&directions, &instructions, boxToDoorCol);
	PositionReadjusted(&directions, &instructions, boxToDoorRow);
	VertPushToBox (&directions, &instructions, boxToDoorRow);

	// Movement Loop
	while (RobotLiveVec[thisIndex]) {
		if (instructions[0] == 'M') {
			move(directions[0], thisIndex);
			fileMutex.lock();
			outFile << "robot " << thisIndex << " move " <<  directions[0] << "\n";
			fileMutex.unlock();
		} else {
			push(directions[0], thisIndex);
			fileMutex.lock();
			outFile << "robot " << thisIndex << " push " <<  directions[0] << "\n";
			fileMutex.unlock();
		}
		instructions.erase(instructions.begin());
		directions.erase(directions.begin());

		if (instructions.empty()) {
			RobotLiveVec[thisIndex] = false;
		}

		std::this_thread::sleep_for(std::chrono::microseconds(robotSleepTime));
	}
	
	return nullptr;
}
// --------------------------------------------------------------------------------------------------------------

//	Rather that writing a function that prints out only to the terminal
//	and then
//		a. restricts me to a terminal-bound app;
//		b. forces me to duplicate the code if I also want to output
//			my grid to a file,
//	I have two options for a "does it all" function:
//		1. Use the stream class inheritance structure (the terminal is
//			an iostream, an output file is an ofstream, etc.)
//		2. Produce an output file and let the caller decide what they
//			want to do with it.
//	I said that I didn't want this course to do too much OOP (and, to be honest,
//	I have never looked seriously at the "stream" class hierarchy), so we will
//	go for the second solution.
string printGrid(void)
{
	//	some ugly hard-coded stuff
	static string doorStr[] = {"D0", "D1", "D2", "D3", "DD4", "D5", "D6", "D7", "D8", "D9"};
	static string robotStr[] = {"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"};
	static string boxStr[] = {"b0", "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9"};
	
	if (numDoors > 10 || numBoxes > 10)
	{
		cout << "This function only works for small numbers of doors and robots" << endl;
		exit(1);
	}

	//	I use sparse storage for my grid
	map<int, map<int, string> > strGrid;
	
	//	add doors
	for (int k=0; k<numDoors; k++)
	{
		strGrid[doorLoc[k].row][doorLoc[k].col] = doorStr[k];
	}
	//	add boxes
	for (int k=0; k<numBoxes; k++)
	{
		strGrid[boxLoc[k].row][boxLoc[k].col] = boxStr[k];
		strGrid[robotLoc[k].row][robotLoc[k].col] = robotStr[k];
	}
	
	ostringstream outStream;

	//	print top border
	printHorizontalBorder(outStream);
	
	for (int i=0; i<numRows; i++)
	{
		outStream << "|";
		for (int j=0; j<numCols; j++)
		{
			if (strGrid[i][j].length() > 0)
				outStream << " " << strGrid[i][j];
			else {
				outStream << " . ";
			}
		}
		outStream << "|" << endl;
	}
	//	print bottom border
	printHorizontalBorder(outStream);

	strGrid.clear();
	return outStream.str();
}

void printHorizontalBorder(ostringstream& outStream)
{
	outStream << "+--";
	for (int j=1; j<numCols; j++)
	{
		outStream << "---";
	}
	outStream << "-+" << endl;
}


void generatePartitions(void)
{
	const unsigned int NUM_PARTS = (numCols+numRows)/4;

	//	I decide that a partition length  cannot be less than 3  and not more than
	//	1/4 the grid dimension in its Direction
	const unsigned int MIN_PARTITION_LENGTH = 3;
	const unsigned int MAX_HORIZ_PART_LENGTH = numCols / 4;
	const unsigned int MAX_VERT_PART_LENGTH = numRows / 4;
	const unsigned int MAX_NUM_TRIES = 20;
	uniform_int_distribution<unsigned int> horizPartLengthDist(MIN_PARTITION_LENGTH, MAX_HORIZ_PART_LENGTH);
	uniform_int_distribution<unsigned int> vertPartLengthDist(MIN_PARTITION_LENGTH, MAX_VERT_PART_LENGTH);
	uniform_int_distribution<unsigned int> rowDist(1, numRows-2);
	uniform_int_distribution<unsigned int> colDist(1, numCols-2);

	for (unsigned int w=0; w< NUM_PARTS; w++)
	{
		//	Case of a vertical partition
		if (headsOrTails(engine))
		{
			bool goodPart = false;

			//	I try a few times before giving up
			for (unsigned int k=0; k<MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a column index
				unsigned int col = colDist(engine);
				unsigned int length = vertPartLengthDist(engine);

				//	now a random start row
				unsigned int startRow = 1 + rowDist(engine) % (numRows-length-1);
				for (unsigned int row=startRow, i=0; i<length && goodPart; i++, row++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the partition is possible,
				if (goodPart)
				{
					//	add it to the grid and to the partition list
					SlidingPartition part;
					part.isVertical = true;
					for (unsigned int row=startRow, i=0; i<length && goodPart; i++, row++)
					{
						grid[row][col] = SquareType::VERTICAL_PARTITION;
						GridPosition pos = {row, col};
						part.blockList.push_back(pos);
					}

					partitionList.push_back(part);
				}
			}
		}
		// case of a horizontal partition
		else
		{
			bool goodPart = false;

			//	I try a few times before giving up
			for (unsigned int k=0; k<MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a row index
				unsigned int row = rowDist(engine);
				unsigned int length = vertPartLengthDist(engine);

				//	now a random start row
				unsigned int startCol = 1 + colDist(engine) % (numCols-length-1);
				for (unsigned int col=startCol, i=0; i<length && goodPart; i++, col++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the wall first, add it to the grid and build SlidingPartition object
				if (goodPart)
				{
					SlidingPartition part;
					part.isVertical = false;
					for (unsigned int col=startCol, i=0; i<length && goodPart; i++, col++)
					{
						grid[row][col] = SquareType::HORIZONTAL_PARTITION;
						GridPosition pos = {row, col};
						part.blockList.push_back(pos);
					}

					partitionList.push_back(part);
				}
			}
		}
	}
}

