/*
 * Map.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: user
 */

#include "Map.h"

Map::Map(OccupancyGrid &grid) :grid(grid) {
	cv::namedWindow("OccupancyGrid-view");

	config = Configuration::Instance();

	robotHeightInPixels = config.getHeight() / grid.getResolution();
	robotWidthInPixels = config.getWidth() / grid.getResolution();

	convertToCoarseGrid();

	//initMat(*coarseGrid);
	initMat(getBlownGrid(grid, config->robotSize().height, config->robotSize().width));
}

void Map::initMat(OccupancyGrid &grid) {
	mat = cv::Mat(grid.getHeight(), grid.getWidth(), CV_8UC3);

	cout << "Grid size: " << grid.getHeight() << " * " << grid.getWidth() << endl;
	for (int i=0; i<grid.getHeight(); i++) {
		for (int j=0; j<grid.getWidth(); j++) {
			initCell(grid,i,j);
		}
	}
}

void Map::initCell (OccupancyGrid &grid, int i, int j) {
	Cell cell = grid.getCell(i, j);
	int pixel;

	if (cell == CELL_FREE) pixel = 255;
	else if (cell == CELL_OCCUPIED) pixel = 0;
	else pixel = 128;

	paintCell(i, j, pixel);
}

void Map::paintCell (int i, int j, int pixelR, int pixelG, int pixelB) {
	mat.at<cv::Vec3b>(i,j)[0] = pixelB;
	mat.at<cv::Vec3b>(i,j)[1] = pixelG;
	mat.at<cv::Vec3b>(i,j)[2] = pixelR;
}

void Map::paintCell (int i, int j, int pixel) {
	paintCell(i, j, pixel, pixel, pixel);
}

void Map::show (){
	
	cv::imshow("OccupancyGrid-view", mat);
	cv::waitKey(1);
}

void Map::convertToCoarseGrid() {

	int rows = grid.getHeight() / robotSizeInPixels;
	int cols = grid.getWidth() / robotSizeInPixels;
	double resolution = grid.getResolution() * robotSizeInPixels;

	coarseGrid = new OccupancyGrid(rows, cols, resolution);

	// i and j go through the new grid
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int row = i * robotSizeInPixels;
			int col = j * robotSizeInPixels;

			bool isOccupied = false;

			// k and m go through the origin grid
			for (int k = row; k < row + robotSizeInPixels && !isOccupied; k++) {
				for (int m = col; m < col + robotSizeInPixels; m++) {
					if (grid.getCell(k, m) != CELL_FREE) {
						isOccupied = true;
						break;
					}
				}
			}

			if (isOccupied) courseGrid->setCell(i, j, CELL_OCCUPIED);
			else courseGrid->setCell(i, j, CELL_FREE);
		}
	}
}

// The sizes in cm
int getNumOfPixelsToBlow(double mapResolution) {

	// Size in m
	double height = config.getHeight() / 100;
	double width = config.getWidth() / 100;

	double blowSize = max(height, width) / 2;

	return blowSize / mapResolution;
}

OccupancyGrid getBlownGrid(OccupancyGrid grid) {

	int blowRadius = getNumOfPixelsToBlow(grid.getResolution());

	OccupancyGrid blownGrid = grid;

	for (int i = 0; i<grid.getHeight(); i++) {
		for (int j = 0; j<grid.getWidth(); j++) {

			if (grid.getCell(i, j) == CELL_OCCUPIED) {

				for (int i2 = i - blowRadius; i2 <= i + blowRadius; i2++) {
					for (int j2 = j - blowRadius; j2 <= j + blowRadius; j2++) {

						if (i2 >= 0 && i2 < grid.getHeight() && j2 >= 0 && j2 < grid.getWidth()) {
							blownGrid.setCell(i2, j2, CELL_OCCUPIED);
						}
					}
				}
			}
		}
	}

	return blownGrid;
}

Map::~Map() {
	cv::destroyWindow("OccupancyGrid-view");
	delete courseGrid;
}

