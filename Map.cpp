/*
 * Map.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: user
 */

#include "Map.h"

Map::Map(OccupancyGrid &grid) :grid(grid) {
	cv::namedWindow("OccupancyGrid-view");
	initMap();
}

void Map::initMap() {
	mat = cv::Mat(grid.getHeight(), grid.getWidth(), CV_8UC3);

	cout << "Grid size: " << grid.getHeight() << " * " << grid.getWidth() << endl;
	for (int i=0; i<grid.getHeight(); i++) {
		for (int j=0; j<grid.getWidth(); j++) {
			initCell(i,j);
		}
	}
}

void Map::initCell (int i, int j) {
	Cell cell = grid.getCell(i, j);
	int pixel;
	if (cell == CELL_FREE) {
		pixel = 255;
//		if (i<400)cout << i << " * " << j << endl;

	}
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

void Map::show () const{
	cv::imshow("OccupancyGrid-view", mat);
	cv::waitKey(1);
}

Map::~Map() {
	cv::destroyWindow("OccupancyGrid-view");
}

