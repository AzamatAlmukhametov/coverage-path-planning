#include "PathTransformer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

#include "DistanceTransformer.hpp"
#include "ObstacleTransformer.hpp"

using CellMap = std::vector<std::vector<Cell>>;

PathTransformer::PathTransformer(const CellMap& cellMap,
                                 const Coord<int>& start) {
    dtMap = DistanceTransformer().transform(cellMap, start);
    otMap = ObstacleTransformer().transform(cellMap);
}

int PathTransformer::findMaxWeight(const CellMap& map) const {
    int maxWeight = 0;
    for (auto row : map) {
        for (auto el : row) {
            maxWeight = std::max(maxWeight, el.weight);
        }
    }

    return maxWeight;
}

CellMap PathTransformer::transform() const {
    if (dtMap.empty() || otMap.empty()) {
        return {};
    }

    assert(dtMap.size() == otMap.size());
    assert(dtMap[0].size() == otMap[0].size());

    const int rows = dtMap.size();
    const int cols = rows > 0 ? dtMap[0].size() : 0;
    CellMap map = CellMap(rows, std::vector<Cell>(cols));

    const int otMaxWeight = findMaxWeight(otMap);
    const int dtMaxWeight = findMaxWeight(dtMap);
    const double scaler = dtMaxWeight == 0 ? 1 : (double)otMaxWeight / dtMaxWeight;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            map[y][x].type = dtMap[y][x].type;
            
            if (dtMap[y][x].type == cell::OBSTACLE) continue;

            assert(otMap[y][x].weight != 0);
            map[y][x].weight = (int)round(scaler * (dtMulti * dtMap[y][x].weight)) +
                               otMulti * otMaxWeight / otMap[y][x].weight;
        }
    }

    return map;
}