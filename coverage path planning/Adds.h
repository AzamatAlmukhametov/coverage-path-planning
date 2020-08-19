#pragma once

#include <vector>
#include <iostream>
#include <iomanip>
#include "bitmap_image.hpp"

struct Coord
{
	int x;
	int y;

	Coord(int x, int y) : x(x), y(y) {};
	Coord() { x = 0; y = 0; };
	Coord(const Coord& a) : x(a.x), y(a.y) {};

	void reInit(int x, int y)
	{
		this->x = x;
		this->y = y;
	};

	Coord& operator=(const Coord& a)
	{
		x = a.x;
		y = a.y;

		return *this;
	}

	Coord operator+(const Coord& a) const
	{
		return Coord{ x + a.x, y + a.y };
	}

	Coord operator-(const Coord& a) const
	{
		return Coord{ x - a.x, y - a.y };
	}

	bool operator==(const Coord& a) const
	{
		return (x == a.x && y == a.y);
	}
};

struct Node
{
	int x;
	int y;
	float value;

	void reInit(int x, int y, float value)
	{
		this->x = x;
		this->y = y;
		this->value = value;
	};

	Node& operator=(const Node& a)
	{
		x = a.x;
		y = a.y;
		value = a.value;

		return *this;
	}

	Node operator+(const Node& a) const
	{
		return Node{ x + a.x, y + a.y, value + a.value };
	}
};

struct Rect
{
	unsigned int x1, y1, x2, y2;
};

struct MapElem
{
	int type;
	float value;
	float f;
};

struct MovePath
{
	float length;

	std::vector<Coord> pathCoords;

	MovePath() : length(0) {};

	MovePath(float length) : length(length) {};

	MovePath(const MovePath& a)
	{
		length = a.length;
		pathCoords = a.pathCoords;
	}

	void reInit(std::vector<Coord>& coords)
	{
		size_t coordsVectSize = coords.size();
		this->length = 0;
		for (int i = 0; i < coordsVectSize - 1; i++)
		{
			Coord v(coords[i + 1].x - coords[i].x,
				    coords[i + 1].y - coords[i].y);

			this->length += sqrtf(powf(static_cast<float>(v.x), 2) + 
								  powf(static_cast<float>(v.y), 2));
		}

		pathCoords = coords;
	}

	void add(MovePath& movePath)
	{
		size_t pathCoordsVectSize = movePath.pathCoords.size();

		for (int i = 0; i < pathCoordsVectSize - 1; i++)
		{
			Coord v(movePath.pathCoords[i + 1].x - movePath.pathCoords[i].x,
				    movePath.pathCoords[i + 1].y - movePath.pathCoords[i].y);

			this->length += sqrtf(powf(static_cast<float>(v.x), 2) + 
								  powf(static_cast<float>(v.y), 2));
		}

		for (int i = 1; i < pathCoordsVectSize; i++)
			pathCoords.push_back(movePath.pathCoords[i]);
	}
};

class Path
{
private:
	float lengthMWeight = 1;
	float turnsMWeight = 2;

	void reComputeCost()
	{
		this->cost = lengthMWeight * this->lengthM + turnsMWeight * this->turnsM;
	};

public:
	float lengthM;		//1 = 1 moveGridMap square cell size
	int turnsM;			//1 turn = 45degree
	int paintedAreaR;	//1 = 1 routeGridMap cell
	float cost;

	std::vector<Coord> routeCoords;
	std::vector<Coord> moveCoords;

	void init(std::vector<Coord>& routeCoords,
			  std::vector<Coord>& moveCoords);

	Path() :
		lengthM(0),
		turnsM(0),
		paintedAreaR(0),
		cost(0) {};

	Path(const Path&);

	Path(const Path&&);

	Path& operator=(const Path&);

	void set_costFunctionWeights(float lengthMWeight, float turnsMWeight)
	{
		this->lengthMWeight = lengthMWeight;
		this->turnsMWeight = turnsMWeight;

		reComputeCost();
	};

	float get_lengthMWeight() const
	{
		return lengthMWeight;
	};

	float get_turnsMWeight() const
	{
		return turnsMWeight;
	};
};

//structure for parallel computation
struct GridMap
{
	std::vector<std::vector<MapElem>> moveGridMap, routeGridMap;
	bool occup = false;
	Coord robotEnd;

	GridMap(std::vector<std::vector<MapElem>>& moveGridMap,
			std::vector<std::vector<MapElem>>& routeGridMap)
	{
		this->moveGridMap = moveGridMap;
		this->routeGridMap = routeGridMap;
	}
};


void BresenhamCircle(int r, const Coord& base, std::vector<Coord>*);

void formPrintMapf(std::vector<std::vector<MapElem>>&);

const std::vector<Coord> shift4{ { 0, -1 },
								 { 0,  1 },
								 {-1,  0 },
								 { 1,  0 } };

const std::vector<Coord> shift4d{ { 1, -1 },
								  {-1, -1 },
								  {-1,  1 },
								  { 1,  1 } };

const std::vector<Coord> shift8{ Coord{-1, -1 },
								 Coord{ 1, -1 },
								 Coord{-1,  1 },
								 Coord{ 1,  1 },
	  						     Coord{ 0, -1 },
								 Coord{-1,  0 },
								 Coord{ 1,  0 },
								 Coord{ 0,  1 } };

const rgb_t obstColour{ 0, 0, 0 },
			freeSpaceColour{ 255, 255, 255 },
			borderColor{ 0, 255, 0 },
			infColour{ 0, 0, 255 };