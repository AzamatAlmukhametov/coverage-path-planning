#pragma once

#include <vector>
#include <queue>

#include "Coord.hpp"
#include "Types.hpp"

// Create a weight map using wave proragation algorithm.
// The Wave starts at obstacles border coordinates and go through free space.
class ObstacleTransformer {
public:
    using CellMap = std::vector<std::vector<Cell>>;

    CellMap transform(const CellMap& cellMap);

private:
    bool isInImage(const Coord<int>& coord) const;

    void addObstacleBorder(const CellMap& map,
                           std::queue<Coord<int>>& coords,
                           std::vector<std::vector<bool>>& visited) const;

    void updateRowsAndCols(const CellMap& cellMap);

    int rows = 0, cols = 0;
};
