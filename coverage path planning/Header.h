#pragma once

#include <math.h>
#include <string>
#include <set>
#include <queue>
#include <limits>
#include <thread>
#include <mutex>

#include "Adds.h"

/*---------------------------------------------------------------------------
		The general description of the coverage path planning approuch.
	This method uses grid decomposition of the map and wave algorithm.
The route grid map needed for planning areas on the map to visit.
The move grid map needed for planning the approximate trajectory of the robot
moving between the areas. Visiting the areas and transfering between them 
are supposed covering them.
	For planning visiting areas route used the wave algorithm (with no 
obstacle wave summing) by A. Zelinsky1, R.A. Jarvis2, J.C. Byrne2 and 
S. Yuta3 "Planning Paths of Complete Coverage of an Unstructured Environment 
by a Mobile Robot", modified with A* local robot move planning. For the last 
named modification used move grid map decomposition. Obstacle avoidance 
implemented by obstacle inflation algorithm.
	generatePaths function generates robot->end coordinates accordingly to 
density parameter and computates paths for each of them.

		There are parameters and sets:
1) grid cell sizes
	unsigned int routeGridCellWidth_,  routeGridCellHeight_;
	unsigned int moveGridCellSize_; //sqare
2) parameter 0.0...1.0 that cuts paths with 
	covered area < maximum covered area * routeCoverCoefficient_
	float routeCoverCoefficient_;
3) parameter 0.0...1.0 that sets the maximum shift from robot->radius() * 2
in coordinates correction
	float routeCellRadialCorrectionCoeff_;
4) cost function weights
	class Path	float lengthMWeight = 1;
				float turnsMWeight = 2;
5) this length parameter sets the maximum lenght of pretenders path in 
choosing next area for visit
	float maxLengthNearPath = static_cast<float>(2 * robot->radius());
6) integer sets the density of the robot->end coordinates generated on 
route grid map. 
	generatePaths() int density = 4; //Overall 4*4=16 coordinates.
7) (do not change) this set sets the length of arc in inflation algorithm
	drawing arcs float m = static_cast<float>(robot->radius()) / sqrtf(2) - 1;

		Types
1) gridMap[][].type
		//  0 - free space					meshMap()
		// -1 - obstacle					meshMap()
		// -2 - partitially free space		meshMap()
		// -3 - processed by A* algorithm	planLocalPathAStar()
		// -48 - start position				sToEPropWave()
		// -47 - end position				sToEPropWave()
----------------------------------------------------------------------------
*/

class Robot;

class PathPlanner
{
public:

	PathPlanner(std::string filename, Robot&);

	~PathPlanner();

	std::vector<Path> generatePaths();

	Path chooseTheBestPath();

	void markRouteTrajectory();

	void markPathTrajectory();

	void set_costFunctionWeights(float lengthMWeight, float turnsMWeight)
	{
		bestPath.set_costFunctionWeights(lengthMWeight, turnsMWeight);

		for (int i = 0; i < paths.size(); i++)
			paths[i].set_costFunctionWeights(lengthMWeight, turnsMWeight);
	};

	void moveGridCellSize(unsigned int squareSizePx)
	{
		moveGridCellSize_ = squareSizePx;
	};

	void routeGridCellSize(unsigned int squareWidthPx , unsigned int squareHeightPx)
	{
		routeGridCellWidth_ = squareWidthPx;
		routeGridCellHeight_ = squareHeightPx;
	};

	void routeCoverCoefficient(float coeff)
	{
		routeCoverCoefficient_ = coeff;
	};

	void routeCellRadialCorrectionCoeff(float coeff)
	{
		routeCellRadialCorrectionCoeff_ = coeff;
	};
	
	std::vector<Path> get_paths()
	{
		return paths;
	}

private:

	bitmap_image image;

	Robot* robot;

	int frameSize = 2;
	bitmap_image frImg, frMeshImg, frInfImg, frInfMeshImg;
	unsigned int height, width;
	std::vector<std::vector<rgb_t>> frImgMat, frInfImgMat;
	std::vector<Coord> obstBorders;

	unsigned int routeGridCellWidth_,  routeGridCellHeight_;
	unsigned int moveGridCellSize_;
	std::vector<std::vector<MapElem>> routeGridMap,//
									  moveGridMap, moveGridMapBuff;//
	unsigned int routeGridMapRows, routeGridMapCols,
				  moveGridMapRows, moveGridMapCols;

	float routeCoverCoefficient_;
	float routeCellRadialCorrectionCoeff_ ;

	std::vector<Path> paths;//
	Path bestPath;//

	//data for parallel computation
	std::vector<GridMap> dataStorage;

	void markObstacleBorbers();

	void inflateObst();

	void meshMap(bitmap_image& initial, Rect area, Rect gridSize,
				 bitmap_image& res, std::vector<std::vector<MapElem>>& resGridMap);

	void propogateWave(int dataInd);

	Path planPath(int dataInd);

	bool mapCoords(Coord& routeGridMapCoord, Coord& moveGridMapCoord);

	MovePath planLocalPathAStar(Coord, Coord, int dataInd);
};

class Robot
{
private:
	bitmap_image image_;
	unsigned int height_;
	unsigned int width_;
	unsigned int radius_;

public:

	Coord start, end;

	Robot(std::string filename, Coord);

	Robot(const Robot&);

	inline bitmap_image image() const
	{
		return image_;
	}

	inline unsigned int height() const
	{
		return height_;
	}

	inline unsigned int width() const
	{
		return width_;
	}

	inline unsigned int radius() const
	{
		return radius_;
	}
};

