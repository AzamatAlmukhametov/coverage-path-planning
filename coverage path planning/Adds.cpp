#include "Adds.h"

Path::Path(const Path& a)
{
	lengthM = a.lengthM;
	turnsM = a.turnsM;
	paintedAreaR = a.paintedAreaR;
	cost = a.cost;

	routeCoords = a.routeCoords;
	moveCoords = a.moveCoords;
};

Path::Path(const Path&& a)
{
	lengthM = std::move(a.lengthM);
	turnsM = std::move(a.turnsM);
	paintedAreaR = std::move(a.paintedAreaR);
	cost = std::move(a.cost);

	routeCoords = std::move(a.routeCoords);
	moveCoords = std::move(a.moveCoords);
};

void Path::init(std::vector<Coord>& routeCoords,
				std::vector<Coord>& moveCoords)
{
	this->lengthM = 0;
	this->turnsM = 0;
	this->paintedAreaR = 0;
	this->cost = 0;

	const size_t moveCoordSize = moveCoords.size();
	const size_t routeCoordsSize = routeCoords.size();

	if (routeCoordsSize == 0 || moveCoordSize == 0)
		return;

	this->routeCoords = routeCoords;
	this->moveCoords = moveCoords;

	//initializing lengthM and turnsM
	for (int i = 1; i < moveCoordSize - 1; i++)
	{
		Coord prev(moveCoords[i].x - moveCoords[i - 1].x,
				   moveCoords[i].y - moveCoords[i - 1].y),
			  next(moveCoords[i + 1].x - moveCoords[i].x,
				   moveCoords[i + 1].y - moveCoords[i].y);

		float prevLength = sqrtf(powf(static_cast<float>(prev.x), 2) + 
								 powf(static_cast<float>(prev.y), 2));
		float nextLength = sqrtf(powf(static_cast<float>(next.x), 2) + 
								 powf(static_cast<float>(next.y), 2));

		this->lengthM += prevLength;

		if (i == moveCoordSize - 1)
			this->lengthM += nextLength;

		int cos10 = (int)((float)(prev.x * next.x + prev.y * next.y) / 
						  (prevLength * nextLength) * 10);

		if (cos10 == 7) this->turnsM += 1;
		else if (cos10 == 0) this->turnsM += 2;
		else if (cos10 == -7) this->turnsM += 3;
		else if (cos10 == -10) this->turnsM += 4;
	}

	//initializing paintedAreaR
	int intersectionsR = 0;
	int  st = 1;
	for (int i = 0; i < routeCoordsSize; i++)
		if (i - st >= 0 && i + st < routeCoordsSize)
		{
			if (routeCoords[i - st] == routeCoords[i + st])
			{
				intersectionsR++;
				st++;
				i--;
			}
			else
				st = 1;
		}
		else
			st = 1;

	this->paintedAreaR = static_cast<int>(routeCoordsSize) - intersectionsR;

	reComputeCost();
}

Path& Path::operator=(const Path& a)
{
	lengthM = a.lengthM;
	turnsM = a.turnsM;
	paintedAreaR = a.paintedAreaR;
	cost = a.cost;

	routeCoords = a.routeCoords;
	moveCoords = a.moveCoords;

	return *this;
};

void BresenhamCircle(int r, const Coord& base, std::vector<Coord>* coords)
{
	int x = 0, y = r, delta = 2 - 2 * r, error = 0;

	while (y >= 0)
	{
		coords->push_back(Coord{ base.x + x, base.y + y });
		coords->push_back(Coord{ base.x - y, base.y + x });
		coords->push_back(Coord{ base.x - x, base.y - y });
		coords->push_back(Coord{ base.x + y, base.y - x });

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
};

void formPrintMapf(std::vector<std::vector<MapElem>>& gridMap)
{
	int size = 6;

	std::ofstream file("gridmapTrajectory.txt");

	for (auto i : gridMap)
	{
		for (auto j : i)
			if (j.value > 0 && (int)log10((int)j.value) + 2 > size)
				size = (int)log10((int)j.value) + 2;
	}

	for (auto i : gridMap)
	{
		for (auto j : i)
			file << std::setw(size) << (float)(int)(j.value * 10) / 10;
		file << std::endl;
	}
};


