		This program takes all cpu hardware_concurrency() while computation some functions.

		Additional libs:
C++ Bitmap Library (bitmap_image.hpp)
		
		The program is a draft for implementing coverage path planning,
using grid decomposition and wave algorithm. The method consider the initial map as a static 
(not changing environment).

		Test results (see test.. folder) on i3 2350m 2.3Ghz 2 cores 4 threads: 
map2: 2000x1000 ~ 1.30min, 
map9: 3500x2000 ~ 8min

		Input data:
0) robot start position, based on routeGridMap coordinates (should be on free space, 
otherwise it causes crashes).
1) robot.bmp ( >= 80x80px, otherwise it causes errors in inflation algorithm)
2) map.bmp
		Output data:
0) PathPlanner::paths, bestPath 
1) routeGridMap.bmp
2) BorderedImage.bmp
3) test1.bmp, test2.bmp
4) framedBorderedInflatedImage.bmp
5) moveGridMap.bmp
6) path.bmp
7) route.bmp

		The general description of this coverage path planning approuch.
	This method uses grid decomposition of the map and wave algorithm.
The route grid map needed for planning visiting areas on the map.
The move grid map needed for planning the approximate trajectory of the robot,
that will move between the areas. Visiting the areas and transfering between them 
are supposed covering them.
	For planning visiting areas (the route) used the wave algorithm (with no 
obstacle wave summing) by A. Zelinsky1, R.A. Jarvis2, J.C. Byrne2 and 
S. Yuta3 "Planning Paths of Complete Coverage of an Unstructured Environment 
by a Mobile Robot", modified with local robot move planning (A* algorithm). 
For the modification used move grid map decomposition. Obstacle avoidance 
implemented by obstacle inflation algorithm.