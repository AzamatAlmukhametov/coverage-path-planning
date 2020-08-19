#include "Header.h"

PathPlanner::PathPlanner(std::string filename, Robot& robot)
{
	//initial map image reading
	image = bitmap_image(filename);

	if (!image)
	{
		std::cout << "Error - Failed to open: " + filename + "\n";
		return;
	}

	//referencing to the robot
	this->robot = new Robot(robot);

	//setting parameters
	routeGridCellSize(this->robot->width() - 20, this->robot->height() - 20);
	moveGridCellSize(6);
	routeCoverCoefficient(1.0f);
	routeCellRadialCorrectionCoeff(1.0f);
	bestPath.set_costFunctionWeights(1.0f, 2.0f);

    //initializing additional data for path planning
	//creating framed image: avoiding out of range map checking, marking the borders
	frImg.setwidth_height(image.width() + 2 * frameSize, image.height() + 2 * frameSize, true);
	frImg.copy_from(image, frameSize, frameSize);

	width = frImg.width();
	height = frImg.height();

	//forming matrix of framed image for faster accessing
	rgb_t colourB;

	frImgMat.resize(height);
	for (auto& it : frImgMat)
		it.resize(width);

	for (unsigned int i = 0; i < height; i++)
		for (unsigned int j = 0; j < width; j++)
		{
			frImg.get_pixel(j, i, colourB);
			frImgMat[i][j] = colourB;
		}
	//creating the copy of framed image for getiing inflating map
	frInfImgMat = frImgMat;

	//meshing framed map for the route planning, main result: routeGridMap
	//not debug version without saving Img and res img
	frMeshImg = frImg;
	meshMap(frImg, Rect{ 0, 0, width - 1, height - 1 }, Rect{ 0, 0, routeGridCellWidth_ - 1, routeGridCellHeight_ - 1 },
		    frMeshImg, routeGridMap);

	routeGridMapRows = static_cast<unsigned int>(routeGridMap.size());
	if (routeGridMapRows != 0)
		routeGridMapCols = static_cast<unsigned int>(routeGridMap[0].size());
	if (routeGridMapRows == 0 || routeGridMapCols == 0)
	{
		std::cout << "error: routeGridMap is empty" << std::endl;
		return;
	}

	frMeshImg.save_image("routeGridMap.bmp");

	//getting inflated map
	//distance and prescicion: robot->radius() +- 1px
	markObstacleBorbers();
	inflateObst(); //to-do exit from errors

	//meshing framed inflated map for the robot move planning, main result: moveGridMap
	meshMap(frInfImg, Rect{ 0, 0, width - 1, height - 1 }, Rect{ 0, 0, moveGridCellSize_ - 1, moveGridCellSize_ - 1 },
		    frInfMeshImg, moveGridMap);

	moveGridMapRows = static_cast<unsigned int>(moveGridMap.size());
	if (moveGridMapRows != 0)
		moveGridMapCols = static_cast<unsigned int>(moveGridMap[0].size());
	if (moveGridMapRows == 0 || moveGridMapCols == 0)
	{
		std::cout << "error: moveGridMap is empty" << std::endl;
		return;//to-do
	}

	frInfMeshImg.save_image("moveGridMap.bmp");

	moveGridMapBuff = moveGridMap;
};

PathPlanner::~PathPlanner()
{
	delete robot;
}

void PathPlanner::markObstacleBorbers()
{
	//used simple shift8 (const variable) filtering 
	//to-do (rewrite to gpu)
	rgb_t colour;
	Coord base, shifted;

	obstBorders.reserve(height * 2 + width * 2);

	for (int i = 0; i < static_cast<int>(routeGridMapRows); i++)
		for (int j = 0; j < static_cast<int>(routeGridMapCols); j++)
			if (routeGridMap[i][j].type == -2)
			{
				for (int y = i * routeGridCellHeight_ - 1; y < (i + 1) * static_cast<int>(routeGridCellHeight_) + 1; y++)
					for (int x = j * routeGridCellWidth_ - 1; x < (j + 1) * static_cast<int>(routeGridCellWidth_) + 1; x++)
					{
						if (x >= static_cast<int>(width) || y >= static_cast<int>(height))
							break;

						if (x < 0 || y < 0)
							continue;

						colour = frImgMat[y][x];

						if (colour == freeSpaceColour)
						{
							base = Coord{ x, y };

							for (const auto& musk : shift8)
							{
								shifted = base + musk;
								colour = frImgMat[shifted.y][shifted.x];

								if (colour == obstColour)
								{
									obstBorders.push_back({ x, y });
									frImgMat[y][x] = borderColor;
									break;
								}

							}

						}
					}

			}
			else if (routeGridMap[i][j].type == -1)
			{
				for (int y = i * routeGridCellHeight_ - 1; y < (i + 1) * static_cast<int>(routeGridCellHeight_) + 1; y++)
					for (int x = j * routeGridCellWidth_ - 1; x < (j + 1) * static_cast<int>(routeGridCellWidth_) + 1; x++)
					{
						if (x >= static_cast<int>(width) || y >= static_cast<int>(height))
							break;

						if (x < 0 || y < 0)
							continue;

						if (y != (i * routeGridCellHeight_) - 1 &&
							x != (j * routeGridCellWidth_) - 1 &&
							y != (i + 1) * routeGridCellHeight_ &&
							x != (j + 1) * routeGridCellWidth_ )
							continue;

						colour = frImgMat[y][x];

						if (colour == freeSpaceColour)
						{
							base = Coord{ x, y };

							for (const auto& musk : shift8)
							{
								shifted = base + musk;
								colour = frImgMat[shifted.y][shifted.x];

								if (colour == obstColour)
								{
									obstBorders.push_back({ x, y });
									frImgMat[y][x] = borderColor;
									break;
								}

							}

						}
					}

			}

	bitmap_image curr = frImg;

	for (int i = 0; i < static_cast<int>(height); i++)
		for (int j = 0; j < static_cast<int>(width); j++)
		{
			curr.set_pixel(j, i, frImgMat[i][j]);
		}

	curr.save_image("BorderedImage.bmp");
};

void PathPlanner::inflateObst()
{
	//the algorithm description
	//1) getting rough coords of inflated border
	//2) getting precisionly inflated border
	//3) painting space betweet obstacle and precisionly inflated border 

	//1)getting rough coords of inflated border on std::vector<Coord> coords
	Coord base, shifted, shiftedOp, shU, shD, shR, shL;
	std::vector<Coord> coords, coordsBuff;
	coords.reserve(obstBorders.size());
	coordsBuff.reserve(obstBorders.size());

	for (const auto& base : obstBorders)
	{
		shD = base + shift4[0];
		shU = base + shift4[1];
		shL = base + shift4[2];
		shR = base + shift4[3];

		//filter "line"
		//.
		//.
		//.
		if (frImgMat[shU.y][shU.x] == borderColor && frImgMat[shD.y][shD.x] == borderColor)
		{
			int deltaXSign = 0, shiftedX = 0;

			if (frImgMat[shR.y][shR.x] == obstColour ||
				frImgMat[shR.y][shR.x] == borderColor)
				deltaXSign = -1;
			else if (frImgMat[shL.y][shL.x] == obstColour ||
					 frImgMat[shL.y][shL.x] == borderColor)
				deltaXSign = 1;

			shiftedX = base.x + deltaXSign * robot->radius();

			if (shiftedX >= frameSize &&
				shiftedX < static_cast<int>(width) - frameSize &&
				frImgMat[base.y][shiftedX] == freeSpaceColour)
			{
				coords.push_back({ shiftedX, base.y });
				frImgMat[base.y][shiftedX] = infColour;
			}
		}
		//...
		else if (frImgMat[shR.y][shR.x] == borderColor && frImgMat[shL.y][shL.x] == borderColor)
		{
			int deltaYSign = 0, shiftedY = 0;

			if (frImgMat[shD.y][shD.x] == obstColour ||
				frImgMat[shD.y][shD.x] == borderColor)
				deltaYSign = 1;
			else if (frImgMat[shU.y][shU.x] == obstColour ||
					 frImgMat[shU.y][shU.x] == borderColor)
				deltaYSign = -1;

			shiftedY = base.y + deltaYSign * robot->radius();

			if (shiftedY >= frameSize &&
				shiftedY < static_cast<int>(height) - frameSize &&
				frImgMat[shiftedY][base.x] == freeSpaceColour)
			{
				coords.push_back({ base.x, shiftedY });
				frImgMat[shiftedY][base.x] = infColour;
			}
		}
		//diagonal 
		else
		{
			if (frImgMat[shD.y][shD.x] == borderColor &&
				frImgMat[shR.y][shR.x] == borderColor &&
				frImgMat[shD.y][shR.x] == obstColour ||
				frImgMat[shR.y][shR.x] == borderColor &&
				frImgMat[shU.y][shU.x] == borderColor &&
				frImgMat[shU.y][shR.x] == obstColour ||
				frImgMat[shU.y][shU.x] == borderColor &&
				frImgMat[shL.y][shL.x] == borderColor &&
				frImgMat[shU.y][shL.x] == obstColour ||
				frImgMat[shL.y][shL.x] == borderColor &&
				frImgMat[shD.y][shD.x] == borderColor &&
				frImgMat[shD.y][shL.x] == obstColour)
			{
				coordsBuff.push_back(base);
			}
		}
	}

	//drawing arcs
	for (const auto& base : coordsBuff)
	{
		for (const auto& musk : shift4d)
		{
			shifted = base + musk;
			shiftedOp = base - musk;

			if (frImgMat[shifted.y][shifted.x] != obstColour &&
				frImgMat[shiftedOp.y][shiftedOp.x] == obstColour)
			{
				int	x = 0, y = robot->radius(), delta = 2 - 2 * robot->radius(), error = 0;

				float m = static_cast<float>(robot->radius()) / sqrtf(2) - 1;

				while (static_cast<float>(y) >= m)
				{
					Coord curCoord(base.x + musk.x * x, base.y + musk.y * y);

					if (curCoord.y >= frameSize && curCoord.y < static_cast<int>(height) - frameSize &&
						curCoord.x >= frameSize && curCoord.x < static_cast<int>(width) - frameSize &&
						frImgMat[curCoord.y][curCoord.x] == freeSpaceColour)
					{
						coords.push_back(curCoord);
						frImgMat[curCoord.y][curCoord.x] = infColour;
					}

					curCoord.reInit(base.x + musk.x * y, base.y + musk.y * x);

					if (curCoord.y >= frameSize && curCoord.y < static_cast<int>(height) - frameSize &&
						curCoord.x >= frameSize && curCoord.x < static_cast<int>(width) - frameSize &&
						frImgMat[curCoord.y][curCoord.x] == freeSpaceColour)
					{
						coords.push_back(curCoord);
						frImgMat[curCoord.y][curCoord.x] = infColour;
					}

					error = 2 * (delta + y) - 1;

					if (delta < 0 && error <= 0)
					{
						delta += 2 * ++x + 1;
						continue;
					}
					if (delta > 0 && error > 0)
					{
						delta -= 2 * --y + 1;
						continue;
					}
					delta += 2 * (++x - --y);
				}

				break;
			}
		}
	}

	//2) getting precisionly inflated border on frInfImgMat
	//this method based on circle filtering (&innCir) and it requiers 
	//robot->radius() >= 40 

	if (robot->radius() < 40)
	{
		std::cout << "error: low robot radius " << std::endl;
		std::cout << "increase resolution of robot.bmp image to 80x80 at least" << std::endl;
		std::cout << "scale map appropriately" << std::endl;
		return;
	}

	Coord zero = Coord(0, 0);
	std::vector<Coord> innCir;

	BresenhamCircle(static_cast<int>(robot->radius()) - 1, zero, &innCir);

	int curThreadCount = 0;
	std::mutex ctc_mutex;

	auto filterFunc = [this, &coords, &ctc_mutex, &curThreadCount, &innCir](size_t s, size_t e)
	{
		Coord base, shifted;
		bool found;

		auto iterEnd = coords.begin() + e;
		for (auto iter = coords.begin() + s; iter <= iterEnd; ++iter)
		{
			if (iter->y >= frameSize + static_cast<int>(robot->radius()) - 1 &&
				iter->y < static_cast<int>(height) - frameSize - (static_cast<int>(robot->radius()) - 1) &&
				iter->x >= frameSize + static_cast<int>(robot->radius()) - 1 &&
				iter->x < static_cast<int>(width) - frameSize - (static_cast<int>(robot->radius()) - 1))
			{
				base = *iter;
				found = false;

				for (const auto& musk : innCir)
				{
					shifted = base + musk;

					if (frInfImgMat[shifted.y][shifted.x] == obstColour)
					{
						found = true;
						break;
					}
				}

				if (!found)
					frInfImgMat[base.y][base.x] = infColour;
			}
		}

		std::lock_guard<std::mutex> lock(ctc_mutex);
		curThreadCount--;
	};

	int n = std::thread::hardware_concurrency();
	std::thread thread;

	size_t coordsSize = coords.size(),
		   curPackIndex = 0, maxPackCount = n * 2,
		   packSize = coordsSize / maxPackCount;

	//paralleling
	while (curPackIndex < maxPackCount)
	{
		if (curThreadCount < n)
		{
			size_t s = curPackIndex * packSize, e = (curPackIndex + 1) * packSize - 1;

			if (curPackIndex == maxPackCount - 1)
				e = e + (coordsSize % maxPackCount);

			thread = std::thread(filterFunc, s, e);
			thread.detach();

			curPackIndex++;

			std::lock_guard<std::mutex> lock(ctc_mutex);
			curThreadCount++;
		}
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	while (curThreadCount != 0)
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

	frInfImg = frImg; //must to leave this line if test1 and test2 deleted

	//debug here
	//test1: precisionly inflated border
	//test2: roughly inflated border
	for (unsigned int i = 0; i < height; i++)
		for (unsigned int j = 0; j < width; j++)
			frInfImg.set_pixel(j, i, frInfImgMat[i][j]);

	frInfImg.save_image("test1.bmp");

	frInfImg = frImg;

	for (unsigned int i = 0; i < height; i++)
		for (unsigned int j = 0; j < width; j++)
			frInfImg.set_pixel(j, i, frImgMat[i][j]);

	frInfImg.save_image("test2.bmp");
	//end of debug

	//3) painting space betweet obstacle and precisionly inflated border on frInfImgMat
	//used wave method with paralleling in cpu though 
	//to-do (need to rewrite to gpu)
	std::vector<Coord> stack = obstBorders;

	auto funcPaint = [this, &stack, &ctc_mutex, &curThreadCount](size_t s, size_t e)
	{
		Coord base, shifted;

		auto iterEnd = stack.begin() + e, iterBegin = stack.begin() + s;

		std::vector<Coord> stackBuff(iterBegin, iterEnd);

		while (!stackBuff.empty())
		{
			base = stackBuff.back();
			stackBuff.pop_back();

			for (const auto& musk : shift4)
			{
				shifted = base + musk;

				if (frInfImgMat[shifted.y][shifted.x] == freeSpaceColour)
				{
					frInfImgMat[shifted.y][shifted.x] = infColour;
					stackBuff.push_back(shifted);
				}
			}
		}

		std::lock_guard<std::mutex> lock(ctc_mutex);
		curThreadCount--;
	};

	size_t obstBordersSize = obstBorders.size();
	packSize = obstBordersSize / maxPackCount;
	curPackIndex = 0;

	while (curPackIndex < maxPackCount)
	{
		if (curThreadCount < n)
		{
			size_t s = curPackIndex * packSize, e = (curPackIndex + 1) * packSize - 1;

			if (curPackIndex == maxPackCount - 1)
				e = e + (obstBordersSize % maxPackCount);

			thread = std::thread(funcPaint, s, e);
			thread.detach();

			curPackIndex++;

			std::lock_guard<std::mutex> lock(ctc_mutex);
			curThreadCount++;
		}
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	while (curThreadCount != 0)
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

	//getting inflated image on infFrImg and frInfMeshImg(this one for meshing)
	//debug
	frInfImg.setwidth_height(width, height);
	frInfMeshImg.setwidth_height(width, height);

	for (int i = 0; i < static_cast<int>(height); i++)
		for (int j = 0; j < static_cast<int>(width); j++)
		{
			frInfImg.set_pixel(j, i, frInfImgMat[i][j]);
			frInfMeshImg.set_pixel(j, i, frInfImgMat[i][j]);
		}

	frInfImg.save_image("framedBorderedInflatedImage.bmp");
};

void PathPlanner::meshMap(bitmap_image& initial, Rect area, Rect gridSize, 
				          bitmap_image& res, std::vector<std::vector<MapElem>>& resGridMap)
{
	//this algorithm performs a simple grid decomposition of the bitmap_image& initial
	//creating an std::vector<std::vector<MapElem>>& resGridMap (see gridMap[]][.type)
	//creating bitmap_image& res
		// red rectangular - free space
		// yellow rectangular - partitially free space
		// ? - obstacle
	unsigned int meshHeight = gridSize.y2 - gridSize.y1 + 1;
	unsigned int meshWidth = gridSize.x2 - gridSize.x1 + 1;

	size_t rows = height % meshHeight ? height / meshHeight + 1 : height / meshHeight,
		   cols = width % meshWidth ? width / meshWidth + 1 : width / meshWidth;

	resGridMap.resize(rows);

	image_drawer drawRectRed(res), drawRectYel(res);

	drawRectRed.pen_width(1); drawRectYel.pen_width(1);
	drawRectRed.pen_color(255, 0, 0); drawRectYel.pen_color(255, 255, 0);

	rgb_t colour;
	unsigned int fX, fY;
	bool obs = false, blackPi = false, whitePi = false, bluePi = false;

	for (unsigned int y = area.y1; y <= area.y2; y += meshHeight) 
	{
		resGridMap[(y - area.y1) / meshHeight].reserve(cols);

		for (unsigned int x = area.x1; x <= area.x2; x += meshWidth) 
		{

			for (unsigned int i = 0; i < meshHeight; ++i) {
				for (unsigned int j = 0; j < meshWidth; ++j) {

					fX = x + j;
					fY = y + i;

					if (fX > area.x2) break;
					if (fY > area.y2) break;

					if (fX >= width) { fX = width - 1; obs = true; }
					if (fY >= height) { fY = height - 1; obs = true; }

					initial.get_pixel(fX, fY, colour);

					if (colour.red != 255 || colour.green != 255 || colour.blue != 255)
						obs = true;

					if (!blackPi && colour.red == 0 && colour.green == 0 && colour.blue == 0)
						blackPi = true;

					if (!whitePi && colour.red == 255 && colour.green == 255 && colour.blue == 255)
						whitePi = true;

					if (!bluePi && colour.red == 0 && colour.green == 0 && colour.blue == 255)
						bluePi = true;
				}

				if (blackPi && whitePi && bluePi)
					break;
			}

			unsigned int x2 = x + meshWidth - 1; if (x2 > area.x2) x2 = area.x2 - 1;
			unsigned int y2 = y + meshHeight - 1; if (y2 > area.y2) y2 = area.y2 - 1;

			if (obs == false)
			{
				drawRectRed.rectangle(x, y, x2, y2);
				resGridMap[(y - area.y1) / meshHeight].push_back(MapElem{ 0, 0 });
			}
			else if (bluePi && whitePi || blackPi && whitePi || obs && whitePi)
			{
				drawRectYel.rectangle(x, y, x2, y2);
				resGridMap[(y - area.y1) / meshHeight].push_back(MapElem{ -2, 0 });
			}
			else if ((blackPi || bluePi) && !whitePi && obs)
				resGridMap[(y - area.y1) / meshHeight].push_back(MapElem{ -1, 0 });

			obs = false; blackPi = false; bluePi = false; whitePi = false;
		}
	}
};

std::vector<Path> PathPlanner::generatePaths()
{
	int maxThreads = std::thread::hardware_concurrency();
	std::thread thread;
	int curThreadCount = 0;

	//DEBUG
	//maxThreads = 1;

	//initializing data storage for parallel computation
	for (int i = 0; i < maxThreads; i++)
		dataStorage.push_back(GridMap(moveGridMap, routeGridMap));

	std::mutex genData_mutex;

	//initializing computation function
	auto compFunc = [this, &curThreadCount, &genData_mutex](int dataInd)
	{
		//modifing routeGridMap for the planPath function
		propogateWave(dataInd);

		Path bestPath(planPath(dataInd));

		std::lock_guard<std::mutex> lock(genData_mutex);
		paths.push_back(bestPath);
		dataStorage[dataInd].occup = false;
		dataStorage[dataInd].routeGridMap = routeGridMap;
		curThreadCount--;
	};

	//integer sets the density of the robot->end coordinates generated on 
	//route grid map. unsigned int density = 4; Overall 4*4=16 coordinates.
	unsigned int density = 4;

	if (density > routeGridMapCols || density > routeGridMapRows)
	{
		std::cout << "error: density > routeGridCell" << std::endl;
		return std::vector<Path>();
	}

	float stepX = static_cast<float>(routeGridMapCols) / density,
		  stepY = static_cast<float>(routeGridMapRows) / density;
	Coord robotInitialStart = robot->start, robotInitialEnd = robot->end;

	for (float shiftY = stepY; robot->end.y < static_cast<int>(routeGridMapRows);
		robot->end.y = static_cast<int>(shiftY), shiftY += stepY)
	{
		robot->end.x = 0;

		for (float shiftX = stepX; robot->end.x < static_cast<int>(routeGridMapCols);)
		{
			if (robot->start == robot->end)
			{
				robot->end.x = static_cast<int>(shiftX);
				shiftX += stepX;
			}

			if (curThreadCount < maxThreads)
			{
				int dataInd;

				//searching for a data that doesn't uccupied by any other threads
				genData_mutex.lock();
				for (dataInd = 0; dataInd < dataStorage.size(); dataInd++)
					if (!dataStorage[dataInd].occup)
						break;

				dataStorage[dataInd].occup = true;
				dataStorage[dataInd].robotEnd = robot->end;
				genData_mutex.unlock();

				thread = std::thread(compFunc, dataInd);
				thread.detach();

				genData_mutex.lock();
				curThreadCount++;
				genData_mutex.unlock();

				robot->end.x = static_cast<int>(shiftX);
				shiftX += stepX;
			}
			else
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}

	}

	while (curThreadCount != 0)
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

	robot->start = robotInitialStart;
	robot->end = robotInitialEnd;

	return paths;
};

Path PathPlanner::chooseTheBestPath()
{
	//the choise is made by 
	//( covered_area >= (maximum_covered_area * routeCoverCoefficient_))
	// and the minimum Cost, computated in class Path
	if (paths.empty())
	{
		std::cout << "no paths was found" << std::endl;
		return Path();
	}

	float costBuff = std::numeric_limits<float>::max(); //to-do
	int maxCoveredArea = 0;
	int bestPathIndex = 0;

	for (int i = 0; i < paths.size(); i++)
		if (paths[i].paintedAreaR > maxCoveredArea)
			maxCoveredArea = paths[i].paintedAreaR;

	for (int i = 0; i < paths.size(); i++)
		if (paths[i].paintedAreaR >= routeCoverCoefficient_ * maxCoveredArea &&
			paths[i].cost < costBuff)
		{
			costBuff = paths[i].cost;
			bestPathIndex = i;
		}

	bestPath = paths[bestPathIndex];

	return paths[bestPathIndex];
}

void PathPlanner::propogateWave(int dataInd)
{
	//This function uses shift4 sample to propogate the wave from 
	//end position until all the cells will be reached.
	//Each wave step increases its wave front value by 1.

	//referencing to the parallel computation data storage
	std::vector<std::vector<MapElem>>& routeGridMap = dataStorage[dataInd].routeGridMap;
	Coord robotEnd = dataStorage[dataInd].robotEnd;

	//using FIFO data
	std::queue<Coord> cells;
	cells.push(robotEnd);

	Coord base, shifted;

	routeGridMap[robot->start.y][robot->start.x].type = -48;
	routeGridMap[robotEnd.y][robotEnd.x].type = -47;

	while (!cells.empty())
	{
		base = cells.front();
		cells.pop();

		for (const auto& musk : shift4)
		{
			shifted = base + musk;

			if (shifted.y >= 0 && shifted.y < static_cast<int>(routeGridMapRows) &&
				shifted.x >= 0 && shifted.x < static_cast<int>(routeGridMapCols) &&
				routeGridMap[shifted.y][shifted.x].type != -1 &&
				routeGridMap[shifted.y][shifted.x].type != -47 &&
				routeGridMap[shifted.y][shifted.x].value == 0)
			{
				routeGridMap[shifted.y][shifted.x].value = routeGridMap[base.y][base.x].value + 1;
				cells.push(shifted);
			}
		}
	}
};

void PathPlanner::markRouteTrajectory()
{
	if (bestPath.routeCoords.empty())
	{
		std::cout << "no route coords was found" << std::endl;
		return;
	}

	bitmap_image curr = frMeshImg;

	image_drawer drawer(curr);
	drawer.pen_color(255, 0, 255);

	drawer.pen_width(3);
	int eI = static_cast<int>(bestPath.routeCoords.size()) - 1;
	drawer.circle(routeGridCellWidth_ * bestPath.routeCoords[0].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * bestPath.routeCoords[0].y + routeGridCellHeight_ / 2, 5);
	drawer.circle(routeGridCellWidth_ * bestPath.routeCoords[eI].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * bestPath.routeCoords[eI].y + routeGridCellHeight_ / 2, 5);

	drawer.pen_width(1);
	for (int i = 0; i < eI; i++)
	{
		drawer.line_segment(routeGridCellWidth_ * bestPath.routeCoords[i].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * bestPath.routeCoords[i].y + routeGridCellHeight_ / 2,
							routeGridCellWidth_ * bestPath.routeCoords[i + 1].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * bestPath.routeCoords[i + 1].y + routeGridCellHeight_ / 2);
		drawer.circle(routeGridCellWidth_ * bestPath.routeCoords[i + 1].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * bestPath.routeCoords[i + 1].y + routeGridCellHeight_ / 2, 3);
	}

	curr.save_image("route.bmp");
};

void PathPlanner::markPathTrajectory()
{
	if (bestPath.moveCoords.empty())
	{
		std::cout << "no move coords was found" << std::endl;
		return;
	}

	bitmap_image curr = frInfMeshImg;

	image_drawer drawer(curr);
	drawer.pen_color(10, 50, 10);


	drawer.pen_width(3);
	int eI = static_cast<int>(bestPath.moveCoords.size()) - 1;
	drawer.circle(moveGridCellSize_ * bestPath.moveCoords[0].x + moveGridCellSize_ / 2, moveGridCellSize_ * bestPath.moveCoords[0].y + moveGridCellSize_ / 2, moveGridCellSize_ / 2);
	drawer.circle(moveGridCellSize_ * bestPath.moveCoords[eI].x + moveGridCellSize_ / 2, moveGridCellSize_ * bestPath.moveCoords[eI].y + moveGridCellSize_ / 2, moveGridCellSize_ / 2);

	drawer.pen_width(1);
	for (int i = 0; i < eI; i++)
	{
		drawer.line_segment(moveGridCellSize_ * bestPath.moveCoords[i].x + moveGridCellSize_ / 2, moveGridCellSize_ * bestPath.moveCoords[i].y + moveGridCellSize_ / 2,
							moveGridCellSize_ * bestPath.moveCoords[i + 1].x + moveGridCellSize_ / 2, moveGridCellSize_ * bestPath.moveCoords[i + 1].y + moveGridCellSize_ / 2);
		drawer.circle(moveGridCellSize_ * bestPath.moveCoords[i + 1].x + moveGridCellSize_ / 2, moveGridCellSize_ * bestPath.moveCoords[i + 1].y + moveGridCellSize_ / 2, moveGridCellSize_ / 2 - 2);
	}

	curr.save_image("path.bmp");
};

Path PathPlanner::planPath(int dataInd)
{
	//The approuch description.
	//The next route cell prenetders forming with shift4 sample.
	//The cells among the pretenders are choosen by the highest_value:
//routeGridMap[][].value - movePaths[].length / (robot->radius() * 2) * moveGridCellSize_ }).
//highest_value present the next_cell_value minus length_to_it(normalized to robot->radius()*2)
	//Search continues until all the reacheable (see mapCoords function) 
//route cells will be visited.
	//There is a maxLengthNearPath = distance between base coordinate and shifted.
//It is the maximum border for founded paths.
	//Brunches marked by coordsNodes. If a deadlock happening, then the current
//position getting back to the last coordsNodes[].

	//referencing to the parallel computation data storage
	std::vector<std::vector<MapElem>>& routeGridMap = dataStorage[dataInd].routeGridMap;

	//coordNodeOld variable needed to avoid losing route cells that
	//are cut by maxLengthNearPath
	Coord base, shifted , coordNodeOld(robot->start);

	//maxLengthNearPath measures in moveGridCellSize_, becauce of need to 
	//compare with movePaths.back().length
	float maxLengthNearPath = static_cast<float>(3 * robot->radius()) / moveGridCellSize_;

	base = robot->start;

	//represents the whole move path coordinates and length
	MovePath wholePath;

	//represents the whole route coordinates
	std::vector<Coord> route;
	route.push_back(robot->start);

	//to-do a structure: 1 base and 4 shift coords
	//it helps to avoid double computation of mapCoords, planLocalPathAStar in deadlock case
	std::vector<Coord> coordsNodes;

	//while ( route.back().x != robotEnd.x || 
	//		route.back().y != robotEnd.y )
	while (true)
	{
		std::vector<Node> routePretenders;
		std::vector<MovePath> movePaths;

		for (const auto& musk : shift4)
		{
			shifted = base + musk;

			if (shifted.y >= 0 && shifted.y < static_cast<int>(routeGridMapRows) &&
				shifted.x >= 0 && shifted.x < static_cast<int>(routeGridMapCols) &&
				routeGridMap[shifted.y][shifted.x].type != -1)
			{
				Coord moveGridMapLocalStartCoord, moveGridMapLocalEndCoord;

				if (!mapCoords(shifted, moveGridMapLocalEndCoord))
					continue;

				if (base == robot->start && wholePath.pathCoords.empty())
				{
					if (!mapCoords(base, moveGridMapLocalStartCoord))
					{
						std::cout << "Robot start position is wrong" << std::endl;
						break;
					}
					else
					{
						std::vector<Coord> s(1, moveGridMapLocalStartCoord);
						wholePath.reInit(s);
					}
				}
				else if (base == coordNodeOld)
				{
					//deadlock case. Using coordsNodes as a base.
					mapCoords(base, moveGridMapLocalStartCoord);

					MovePath localPath = planLocalPathAStar(moveGridMapLocalStartCoord,
															moveGridMapLocalEndCoord,
															dataInd);

					if (localPath.length > maxLengthNearPath)
						continue;
				}

				moveGridMapLocalStartCoord = wholePath.pathCoords.back();

				MovePath localPath = planLocalPathAStar(moveGridMapLocalStartCoord, 
					      								moveGridMapLocalEndCoord, 
														dataInd);
				
				if (!(base == coordNodeOld) && localPath.length > maxLengthNearPath)
					continue;

				if (!localPath.pathCoords.empty())
					movePaths.push_back(localPath);
				else
					continue;

				routePretenders.push_back({ shifted.x, shifted.y, routeGridMap[shifted.y][shifted.x].value - movePaths.back().length / (robot->radius() * 2) * moveGridCellSize_ });
			}
		}

		routeGridMap[base.y][base.x].type = -1;

		//choosing route from routePretenders
		if (!routePretenders.empty())
		{
			if (routePretenders.size() > 1)
				coordsNodes.push_back(base);

			Node maxElem{ 0, 0, routePretenders.back().value };
			int localPathIndex = static_cast<int>(routePretenders.size()) - 1;

			for (int u = 0; u < routePretenders.size(); u++)
				if (routePretenders[u].value >= maxElem.value)
				{
					maxElem = routePretenders[u];
					localPathIndex = u;
				}

			//the length measures in moveGridCellSize_
			wholePath.add(movePaths[localPathIndex]);
			base.x = maxElem.x;
			base.y = maxElem.y;
			route.push_back(base);
		}
		else
		{
			if (coordsNodes.size() == 0)
				break;

			coordNodeOld = coordsNodes.back();
			base = coordsNodes.back();
			coordsNodes.pop_back();
		}
	}

	//this->bestPath needed for bestPath.set_costFunctionWeights(1.0f, 2.0f);
	Path bestPath = this->bestPath;
	bestPath.init(route, wholePath.pathCoords);

	// output test
	// for testing it need to set maxThreads = 1;
	
	/*
	bitmap_image curr1 = frMeshImg;

	image_drawer drawer1(curr1);
	drawer1.pen_color(255, 0, 255);

	int eI = route.size() - 1;
	if (!route.empty())
	{
		drawer1.pen_width(3);
		drawer1.circle(routeGridCellWidth_ * route[0].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * route[0].y + routeGridCellHeight_ / 2, 5);
		drawer1.circle(routeGridCellWidth_ * route[eI].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * route[eI].y + routeGridCellHeight_ / 2, 5);
	}

	drawer1.pen_width(1);
	for (int i = 0; i < eI; i++)
	{
		drawer1.line_segment(routeGridCellWidth_ * route[i].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * route[i].y + routeGridCellHeight_ / 2,
							 routeGridCellWidth_ * route[i + 1].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * route[i + 1].y + routeGridCellHeight_ / 2);
		drawer1.circle(routeGridCellWidth_ * route[i + 1].x + routeGridCellWidth_ / 2, routeGridCellHeight_ * route[i + 1].y + routeGridCellHeight_ / 2, 3);
	}

	curr1.save_image("routeTrajectory.bmp");
	
	bitmap_image curr2 = frInfMeshImg;

	image_drawer drawer2(curr2);
	drawer2.pen_color(10, 50, 10);

	eI = wholePath.pathCoords.size() - 1;
	if (!wholePath.pathCoords.empty())
	{
		drawer2.pen_width(3);
		drawer2.circle(moveGridCellSize_  * wholePath.pathCoords[0].x + moveGridCellSize_ / 2, moveGridCellSize_  * wholePath.pathCoords[0].y + moveGridCellSize_ / 2, moveGridCellSize_ / 2);
		drawer2.circle(moveGridCellSize_  * wholePath.pathCoords[eI].x + moveGridCellSize_ / 2, moveGridCellSize_  * wholePath.pathCoords[eI].y + moveGridCellSize_ / 2, moveGridCellSize_ / 2);
	}

	drawer2.pen_width(1);
	for (int i = 0; i < eI; i++)
	{
		drawer2.line_segment(moveGridCellSize_ * wholePath.pathCoords[i].x + moveGridCellSize_ / 2, moveGridCellSize_ * wholePath.pathCoords[i].y + moveGridCellSize_ / 2,
							 moveGridCellSize_ * wholePath.pathCoords[i + 1].x + moveGridCellSize_ / 2, moveGridCellSize_ * wholePath.pathCoords[i + 1].y + moveGridCellSize_ / 2);
		drawer2.circle(moveGridCellSize_ * wholePath.pathCoords[i + 1].x + moveGridCellSize_ / 2, moveGridCellSize_ * wholePath.pathCoords[i + 1].y + moveGridCellSize_ / 2, moveGridCellSize_ / 2 - 2);
	}

	curr2.save_image("pathTrajectory.bmp");	
	*/

	return bestPath;
};

bool PathPlanner::mapCoords(Coord& routeGridMapCoord, Coord& moveGridMapCoord)
{
	//This function converts routeGridMapCoord to moveGridMapCoord
	//filter 1: filters out of framed Map coordinates
	//filter 2: filters the center of robot coordinates that cannot 
//be corrected by the nearest available free space area pixels. Maximum
//distance equal to rMax. r step increas equal to moveGridCellSize_.
	//filter 3: filters the coordinates that cannot be mapped 
//to moveGridMap. Used shift8 sample.

	Coord prCoord;

	//filter 1: out of frInfImgMat(framed Map) coordinates
	prCoord = { static_cast<int>(routeGridCellWidth_ * routeGridMapCoord.x + routeGridCellWidth_ / 2),
				static_cast<int>(routeGridCellHeight_ * routeGridMapCoord.y + routeGridCellHeight_ / 2) };

	if (prCoord.y < 0)
		prCoord.y = 0;
	else if (prCoord.y >= static_cast<int>(height))
		prCoord.y = height - 1;

	if (prCoord.x < 0)
		prCoord.x = 0;
	else if (prCoord.x >= static_cast<int>(width))
		prCoord.x = width - 1;

	//filter 2: allowability for the center of the robot
	std::vector<Coord> buffCoords;
	bool replaced = false;

	if (frInfImgMat[prCoord.y][prCoord.x] != freeSpaceColour)
	{
		//optimal route Cell Radial Correction Coefficient
		//float routeCellRadialCorrectionCoeff_ = ((float)(robot->radius() * 2) - moveGridCellSize_) / (robot->radius() * 2);

		float r = static_cast<float>(moveGridCellSize_), 
			  rMax = routeCellRadialCorrectionCoeff_ * robot->radius() * 2;

		for (; !replaced && r <= rMax; r+=moveGridCellSize_)
		{
			BresenhamCircle(static_cast<int>(r), prCoord, &buffCoords);

			for(const auto& coord : buffCoords)
				if (coord.y >= 0 && coord.y < static_cast<int>(height) &&
					coord.x >= 0 && coord.x < static_cast<int>(width))
					if (frInfImgMat[coord.y][coord.x] == freeSpaceColour)
					{
						prCoord = coord;
						replaced = true;
						break;
					}

			buffCoords.clear();
		}

		if (!replaced)
			return false;
	}

	//filter 3: map coord to moveGridMap
	Coord base, shifted;
	bool found = false;

	base = { prCoord.x / static_cast<int>(moveGridCellSize_), prCoord.y / static_cast<int>(moveGridCellSize_) };

	if (moveGridMap[base.y][base.x].type == 0)
		found = true;
	else
	{
		for (auto begin = shift8.begin(), end = shift8.end() - 1; begin != end; end--)
		{
			shifted = base + *end;

			if (shifted.y >= 0 && shifted.y < static_cast<int>(moveGridMapRows) &&
				shifted.x >= 0 && shifted.x < static_cast<int>(moveGridMapCols) &&
				moveGridMap[shifted.y][shifted.x].type == 0)
			{
				found = true;
				base = shifted;
				break;
			}
		}
	}

	if (!found) return false;

	prCoord = base;

	moveGridMapCoord = prCoord;

	return true;
};

MovePath PathPlanner::planLocalPathAStar(Coord start, Coord end, int dataInd)
{
	//This function implements A-Start algorithm. Path choosing based on 
//gready algorithm.
	//The cost function f(x)=g(x)+h(x); 
	//The algorithm starts from the Coord end coordinate.
	//g(x)=the shortest length path from the start(Coord end) to x.
	//h(x)=euclidean_distance from x to the goal point(Coord start).
	//The wave is propogating by shift8 sample.

	//referencing to the parallel computation data storage
	std::vector<std::vector<MapElem>>& moveGridMap = dataStorage[dataInd].moveGridMap;

	//marking changes for faster moveGridMap update
	std::vector<Coord> moveGridMapChanges;
	moveGridMapChanges.push_back(start);
	moveGridMapChanges.push_back(end);

	//1) creating A-star wave starting from end coordinate
	MovePath localPath; //return variable
	Coord base, shifted;

	moveGridMap[end.y][end.x].value = 1;
	moveGridMap[end.y][end.x].f =
		moveGridMap[end.y][end.x].value +
		sqrtf(powf(static_cast<float>(start.x - end.x), 2) +
			  powf(static_cast<float>(start.y - end.y), 2));

	bool(*compFunc)(Node, Node) = [](Node c1, Node c2) 
	{
		return c1.value < c2.value;
	};
	std::multiset<Node, bool(*)(Node, Node)> coordsSet(compFunc);
	coordsSet.insert({ end.x, end.y, moveGridMap[end.y][end.x].f });

	while (!coordsSet.empty() && 
		   !((*coordsSet.begin()).x == start.x && (*coordsSet.begin()).y == start.y))
	{
		base.x = (*coordsSet.begin()).x;
		base.y = (*coordsSet.begin()).y;
		coordsSet.erase(coordsSet.begin());

		for (const auto& musk : shift8)
		{
			shifted = base + musk;

			if (shifted.y >= 0 && shifted.y < static_cast<int>(moveGridMapRows) &&
				shifted.x >= 0 && shifted.x < static_cast<int>(moveGridMapCols) &&
				moveGridMap[shifted.y][shifted.x].type == 0)
			{
				float waveFrontValue = moveGridMap[base.y][base.x].value + 
					sqrtf(static_cast<float>(pow(musk.x, 2) + pow(musk.y, 2)));

				if (moveGridMap[shifted.y][shifted.x].value == 0)
					moveGridMapChanges.push_back(shifted);

				if (moveGridMap[shifted.y][shifted.x].value == 0 ||
					moveGridMap[shifted.y][shifted.x].value > waveFrontValue)
				{
					moveGridMap[shifted.y][shifted.x].value = waveFrontValue;
					moveGridMap[shifted.y][shifted.x].f =
						waveFrontValue +
						sqrtf(static_cast<float>(pow((start.x - shifted.x), 2) +
												 pow((start.y - shifted.y), 2)));
					coordsSet.insert({ shifted.x, shifted.y,  moveGridMap[shifted.y][shifted.x].f });
				}
			}
		}

		moveGridMap[base.y][base.x].type = -3;
	}

	if (coordsSet.empty())
	{
		std::cout << "the wave doesn't reach the end coord" << std::endl;

		//updating moveGridMap
		for (const auto& coord : moveGridMapChanges)
			moveGridMap[coord.y][coord.x] = moveGridMapBuff[coord.y][coord.x];

		return localPath;
	}

	//2) finding one of the path
	//may try another algorithms 
	//(currently - gradient method(only 1 cell) - 
	//see std::vector<float> deltas; variable below)
	std::vector<Coord> stack(1, start); //stack for prototyping

	std::vector<Coord> path(1, start);
	std::vector<Coord> pretends;

	while (!(stack.back() == end))
	{
		base = stack.back();
		stack.pop_back();

		for (const auto& musk : shift8)
		{
			shifted = base + musk;

			if (shifted.y >= 0 && shifted.y < static_cast<int>(moveGridMapRows) &&
				shifted.x >= 0 && shifted.x < static_cast<int>(moveGridMapCols) &&
				moveGridMap[shifted.y][shifted.x].value != 0)
				pretends.push_back(shifted);
		}

		std::vector<float> deltas;

		for (const auto& it : pretends)
		{
			deltas.push_back(moveGridMap[base.y][base.x].value - moveGridMap[it.y][it.x].value -
				sqrtf(powf(static_cast<float>(it.x - base.x), 2) + 
					  powf(static_cast<float>(it.y - base.y), 2)));
		}

		float maxValue = deltas.back();
		Coord maxCell;

		for (int i = 0; i < deltas.size(); i++)
		{
			if (deltas[i] >= maxValue)
			{
				maxValue = deltas[i];
				maxCell = pretends[i];
			}
		}

		stack.push_back(maxCell);
		path.push_back(maxCell);
		moveGridMap[maxCell.y][maxCell.x].value = 0;
		pretends.clear();
	}

	//updating moveGridMap
	for (const auto& coord : moveGridMapChanges)
		moveGridMap[coord.y][coord.x] = moveGridMapBuff[coord.y][coord.x];

	localPath.reInit(path);

	return localPath;
};

Robot::Robot(std::string filename, Coord start)
{
	image_ = bitmap_image(filename);

	if (!image_)
	{
		std::cout << "Error - Failed to open: " + filename + "\n";
		return;
	}

	height_ = image_.height();
	width_ = image_.width();

	//float prescRad = sqrt(pow((float)height / 2, 2) + pow((float)width / 2, 2));
	//radius = (int)(prescRad * 10) % 10 == 0 ? (int)prescRad : (int)prescRad + 1;
	//if 80 < 82 then r = 41, but it occupies 42px. Diameter occupies 42 + 41 px = 83px
	radius_ = height_ < width_ ? (width_ % 2 == 0 ? width_ / 2 : width_ / 2) 
							   : (height_ % 2 == 0 ? height_ / 2 : height_ / 2);

	this->start = start;
	this->end = { 0, 0 };
};

Robot::Robot(const Robot& robot)
{
	image_ = robot.image();
	height_ = robot.height();
	width_ = robot.width();
	radius_ = robot.radius();

	start = robot.start;
	end = robot.end;
}
