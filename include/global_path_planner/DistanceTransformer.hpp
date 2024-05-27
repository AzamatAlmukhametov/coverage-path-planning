#pragma once

#include <vector>
#include <queue>

#include "Coord.hpp"
#include "Types.hpp"

// Create a weight map using wave proragation algorithm.
// The Wave starts at start coordinate and go through free space.
class DistanceTransformer {
public:
    using CellMap = std::vector<std::vector<Cell>>;

    CellMap transform(const CellMap& cellMap, const Coord<int>& start);

private:
    bool isInImage(const Coord<int>& coord) const;

    void updateRowsAndCols(const CellMap& map);

    bool setStartCoord(const CellMap& map, const Coord<int>& startArg);

    int rows = 0, cols = 0;
    Coord<int> start = {0, 0};
};