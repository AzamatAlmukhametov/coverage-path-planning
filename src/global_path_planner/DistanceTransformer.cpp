#include "DistanceTransformer.hpp"

#include <iostream>
#include <stdexcept>

#include "Shifts.hpp"

using CellMap = std::vector<std::vector<Cell>>;

CellMap DistanceTransformer::transform(const CellMap& cellMap,
                                       const Coord<int>& start) {
    std::cout << "DistanceTransformer::transform: starts" << std::endl;

    CellMap map = cellMap;
    
    updateRowsAndCols(map);
    if (!setStartCoord(map, start)) {
        throw std::runtime_error("start coordinate is invalid");
    }

    std::queue<Coord<int>> q;
    q.push(start);

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols));
    visited[start.y][start.x] = true;

    while (!q.empty()) {
        size_t size = q.size();
        while (size--) {
            Coord<int> base = q.front();
            q.pop();

            for (auto sh : shift8) {
                Coord<int> coord = base + sh;

                if (!isInImage(coord)) continue;
                if (map[coord.y][coord.x].type == cell::OBSTACLE) continue;
                if (visited[coord.y][coord.x]) continue;

                map[coord.y][coord.x].weight = map[base.y][base.x].weight + 1;
                visited[coord.y][coord.x] = true;
                q.push(coord);
            }
        }
    }

    std::cout << "DistanceTransformer::transform: successfull" << std::endl;
    return map;
}

bool DistanceTransformer::isInImage(const Coord<int>& coord) const {
    return coord.y >= 0 && coord.y < rows &&
           coord.x >= 0 && coord.x < cols;
}

void DistanceTransformer::updateRowsAndCols(const CellMap& map) {
    rows = map.size();
    cols = rows > 0 ? map[0].size() : 0;
}

bool DistanceTransformer::setStartCoord(const CellMap& map,
                                        const Coord<int>& startArg) {
    if (isInImage(startArg) &&
        map[startArg.y][startArg.x].type != cell::OBSTACLE) {
        start = startArg;
        return true;
    }

    return false;
}