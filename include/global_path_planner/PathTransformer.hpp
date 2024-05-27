#pragma once

#include <vector>

#include "Coord.hpp"
#include "Types.hpp"

// Create a weight map using costfunction with the following arguments:
//      distance transformer map (dtMap)
//      obstacle transformer map (otMap)
//      max weight in obstacle transformer map (maxWeight)
// Costfunciton:
//      const double scaler = dtMaxWeight == 0 ? 1 : (double)otMaxWeight / dtMaxWeight;
//      map[y][x].weight = (int)round(scaler * (dtMulti * dtMap[y][x].weight)) +
//                         otMulti * otMaxWeight / otMap[y][x].weight;
class PathTransformer {
public:
    using CellMap = std::vector<std::vector<Cell>>;

    PathTransformer(const CellMap& cellMap, const Coord<int>& start);

    CellMap transform() const;

    void setObstacleTransformerMultiplier(int multiplier) {
        otMulti = multiplier;
    }

    void setDistanceTransformerMultiplier(int multiplier) {
        dtMulti = multiplier;
    }

private:

    int findMaxWeight(const CellMap& map) const;

    CellMap otMap;
    CellMap dtMap;
    int otMulti = 1;
    int dtMulti = 1;
};