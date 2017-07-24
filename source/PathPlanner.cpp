/*
 * PathPlanner.cpp
 *
 *  Created on: Jun 17, 2017
 *      Author: user
 */

#include "../headers/PathPlanner.h"

PathPlanner::PathPlanner(OccupancyGrid& grid, int startRow, int startCol) :
	grid(grid), startRow(startRow), startCol(startCol){
}

PathPlanner::~PathPlanner() {
	// TODO Auto-generated destructor stub
}

void PathPlanner::buildGraph() {

	int rows = grid.getHeight();
	int cols = grid.getWidth();

	mat.resize(rows);
	for (int i = 0; i < rows; ++i) {
		mat[i].resize(cols);
	}

	// Running over the grid
	for (int i = 0;  i <  rows; ++i) {
		for (int j = 0; j <  cols; ++j) {

			// Getting current cell
			Cell c = grid.getCell(i, j);

			// Initializing node if the is free
			if (c == CELL_FREE) {

				Node* n = new Node();
				n->row = i;
				n->col = j;
				n->f = FLT_MAX;
				n->g = FLT_MAX;
				n->h = FLT_MAX;
				n->parent = new Node;
				n->parent->row = -1;
				n->parent->col = -1;

				mat[i][j] = n;
			}
		}
	}
}

struct NodeCostComparator {
	bool operator() (const Node* n1, const Node* n2) {
		return n1->f > n2->f;
	}
};

vector<Node*> PathPlanner::getSuccessors(Node* node) {
	int row = node->row;
	int col = node->col;

	vector<Node*> successors;

	// Upper
	if (row < grid.getHeight() - 1 && mat[row + 1][col]) {
		successors.push_back(mat[row + 1][col]);
	}

	// Lower
	if (row > 0 && mat[row -1][col]) {
		successors.push_back(mat[row -1][col]);
	}

	// Right
	if (col < grid.getWidth() - 1 && mat[row][col + 1]) {
		successors.push_back(mat[row][col + 1]);
	}

	// Left
	if (col < 0 && mat[row][col - 1]) {
		successors.push_back(mat[row][col - 1]);
	}

	return successors;
}

bool PathPlanner::areNodesEqual(Node* n1, Node* n2) {
	return n1->col == n2->col && n1->row == n2->row;
}

// A Utility Function to calculate the 'h' heuristics.
double PathPlanner::calculateHValue(Node* source, Node* dest)
{
    // Return using the distance formula
    return ((double)sqrt (
    			(source->row - dest->row) *( source->row - dest->row) +
				(source->col - dest->col)*(source->col-dest->col)
			));
}

Path PathPlanner::reconstructPath(Node* dest) {

	int row = dest->row;
	int col = dest->col;

	Path toRet;
	Node* currParent = dest->parent;

	// We intialized the source node to point to itself as parent
	// so this is the stopping condition
	while(!(currParent->row == row &&
			currParent->col == col)) {

		toRet.push_back(make_pair(row, col));
		row = currParent->row;
		col = currParent->col;
		currParent = currParent->parent;
	}

//	// Printing
//	for (int i = 0; i < toRet.size(); ++i) {
//		cout << "(" << toRet[toRet.size() - 1 - i].first << ", " << toRet[toRet.size() - 1 - i].second << ") -> ";
//	}

	cout << "destination";
	return toRet;
}

Path PathPlanner::computeShortestPath(int destRow, int destCol) {

	buildGraph();

	// Initializing open list
	priority_queue<Node*, vector<Node*>, NodeCostComparator> openList;

	// Initializing closed list
	int rows = grid.getHeight();
	int cols = grid.getWidth();
	bool** closedList = new bool*[grid.getHeight()];
	for (int i = 0; i < rows; ++i) {
		closedList[i] = new bool[cols];
		for (int j = 0; j < cols; j++) {
			closedList[i][j] = false;
		}
	}

	// Initializing first node
	Node* startNode = mat[startRow][startCol];
	startNode->f = 0;
	startNode->g = 0;
	startNode->h = 0;
	startNode->parent->row = startRow;
	startNode->parent->col = startCol;

	// Getting destination node
	Node* dest = mat[destRow][destCol];

	openList.push(startNode);

	Path path;
	bool destinationFound = false;

	while (!openList.empty()) {
		Node *currNode = openList.top();
		openList.pop();
		vector<Node*> successors = getSuccessors(currNode);

		int row = currNode->row;
		int col = currNode->col;

		// Closing current node
		closedList[row][col] = true;

		double gNew, hNew, fNew;

		// Iterating over successors
		vector<Node*>::iterator i = successors.begin();
		for (; i != successors.end(); ++i) {

			Node* currSuccessor = (*i);

			// Checking if current successor is destination
			if (areNodesEqual(currSuccessor, dest)) {

				// Setting parent
				currSuccessor->parent = currNode;
				destinationFound = true;

				return reconstructPath(dest);
			}
			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (!closedList[currSuccessor->row][currSuccessor->col] &&
				mat[currSuccessor->row][currSuccessor->col]) {
				gNew = currNode->g + 1.0;
				hNew = calculateHValue(currSuccessor, dest);
				fNew = gNew + hNew;

				if (currSuccessor->f == FLT_MAX ||
					currSuccessor->f > fNew) {

					currSuccessor->f = fNew;
					currSuccessor->g = gNew;
					currSuccessor->h = hNew;
					currSuccessor->parent = currNode;

					openList.push(currSuccessor);
				}
			}
		}
	}

	if (!destinationFound) {
		cout << "error during calculating path" << endl;
		return path;
	}
}
