//Next steps
	//before and after covering map: 
//writing coordinates into file and simulate the robot area covering movements
//with them
	//tests: maps with noise
	//for better performance to-do a structure: 1 base and 4 shift coords
	//formPrintMapf working not correclty

//paralleling: 
	//1) volitile - different threads can modify the value
	//2) way to optimize computation: creating the routeToMoveGridMap
	//may be special function to transfer route coords into move coords through mapFunction

//General
	//1) planLocalPathAStar -> finding one of the path
// use only greedy algorithm, doesn't count brunching
	//2) planPath doesn't count brunching
//(no solving the task of finding the best path on the graph with lowest 
//length of whole path, only greedy algorithm)
	//2) std::cout << "Robot start is wrong" << std::endl; + exit or?
	//3) adding const in parameters of functions
	//4) not so effective updateGridMap function for big maps with a lot of obstacles
	// changing to -> std::vector<Coord> routeGridMapChanges updating
	//7) adding consts to const functions
	//8) avoid out of map range checking if it is possiable
	//9) add default parameters
	//11) moveGridMapVectRows make const?

//User interface
	//1) adding logs of progress
	//2) how to set start coords in convenient way?

//1) DEBUG   frInfImg.save_image("test1.bmp");

//Possibilities for optimization
	//1) set_pixel is not optimal way to copy rgb_t Mat to bitmap_image

//List of parameters
	//1) float maxLengthNearPath = static_cast<float>(2 * robot->radius());
	//2) class Path	float lengthMWeight = 1;
	//				float turnsMWeight = 2;
	//3) unsigned int routeGridCellWidth_,  routeGridCellHeight_;
	// 	 unsigned int moveGridCellSize_;
	//4) float routeCoverCoefficient_;
	//	 float routeCellRadialCorrectionCoeff_;
	//5) generatePaths() int density = 4;
	//6) drawing arcs float m = static_cast<float>(robot->radius()) / sqrtf(2) - 1;

#include "Header.h"

int main()
{
	Robot robot("robot.bmp", Coord{ 1, 1 });

	PathPlanner planner("map.bmp", robot);

	planner.generatePaths();

	planner.chooseTheBestPath();

	planner.markRouteTrajectory();
	planner.markPathTrajectory();

	planner.set_costFunctionWeights(2, 1);

	planner.chooseTheBestPath();

	planner.markRouteTrajectory();
	planner.markPathTrajectory();
}
