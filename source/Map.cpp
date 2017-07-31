/*
 * Map.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: user
 */

#include "../headers/Map.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

Map::Map(OccupancyGrid &grid) {
	cv::namedWindow("OccupancyGrid-view");

	config = ConfigurationManager::Instance();

	// Blow the grid
	OccupancyGrid blownGrid = getBlownGrid(grid);

	// Init the mat by the grid values
	initMat(blownGrid);

	// Rotate the mat by -30 degrees
	rotateMat();
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

void Map::rotateMat() {

	cv::Mat resultMat = cv::Mat::zeros(mat.rows, mat.cols, mat.type());

	Point2f center = Point2f( mat.cols/2, mat.rows/2 );
	double angle = -30.0;
	double scale = 1.0;

	Mat rotatedMat = cv::getRotationMatrix2D(center, angle, scale);
	cv::warpAffine(mat, resultMat, rotatedMat, resultMat.size());

	mat = resultMat;
}

void Map::initCell (OccupancyGrid &grid, int i, int j) {
	Cell cell = grid.getCell(i, j);
	int pixel;

	if (cell == CELL_FREE) pixel = 255;
	else if (cell == CELL_OCCUPIED) pixel = 0;
	else pixel = 128;

	paintCell(i, j, pixel);
}

Cell Map::getCell(int row, int col) {
	if (mat.at<cv::Vec3b>(row,col)[0] == 255) return CELL_FREE;
	else if (mat.at<cv::Vec3b>(row,col)[0] == 0) return CELL_OCCUPIED;
	else return CELL_UNKNOWN;
}

int Map::getHeight() { return mat.rows; }
int Map::getWidth() { return mat.cols; }

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

// The sizes in cm
int Map::getNumOfPixelsToBlow(double mapResolution, int robotHeight, int robotWidth) {

	// Size in m
	double height = robotHeight / 100.0;
	double width = robotWidth / 100.0;

	double blowSize = max(height, width) / 2;

	return blowSize / mapResolution;
}

OccupancyGrid Map::getBlownGrid(OccupancyGrid grid) {

	int blowRadius = getNumOfPixelsToBlow(grid.getResolution(),
			config->robotSize().height,
			config->robotSize().width);

	OccupancyGrid* blownGrid = new OccupancyGrid(grid);

	for (int i = 0; i<grid.getHeight(); i++) {
		for (int j = 0; j<grid.getWidth(); j++) {

			if (grid.getCell(i, j) == CELL_OCCUPIED) {

				for (int i2 = i - blowRadius; i2 <= i + blowRadius; i2++) {
					for (int j2 = j - blowRadius; j2 <= j + blowRadius; j2++) {

						if (i2 >= 0 && i2 < grid.getHeight() && j2 >= 0 && j2 < grid.getWidth()) {
							blownGrid->setCell(i2, j2, CELL_OCCUPIED);
						}
					}
				}
			}
		}
	}

	return *blownGrid;
}

Map::~Map() {
	cv::destroyWindow("OccupancyGrid-view");
}

