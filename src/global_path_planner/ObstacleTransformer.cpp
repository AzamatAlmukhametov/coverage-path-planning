#include "ObstacleTransformer.hpp"

#include <iostream>

#include "Shifts.hpp"

using CellMap = std::vector<std::vector<Cell>>;

CellMap ObstacleTransformer::transform(const CellMap& cellMap) {
    std::cout << "ObstacleTransformer::transform: starts" << std::endl;
    
    CellMap map = cellMap;
    
    updateRowsAndCols(map);

    std::queue<Coord<int>> q;
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols));
    addObstacleBorder(map, q, visited);

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

    std::cout << "ObstacleTransformer::transform: successfull" << std::endl;
    return map;
}


bool ObstacleTransformer::isInImage(const Coord<int>& coord) const {
    return coord.y >= 0 && coord.y < rows &&
           coord.x >= 0 && coord.x < cols;
}

void ObstacleTransformer::addObstacleBorder(const CellMap& map,
                                            std::queue<Coord<int>>& coords,
                                            std::vector<std::vector<bool>>& visited) const {
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (map[y][x].type != cell::OBSTACLE) continue;

            for (auto sh : shift8) {
                Coord<int> coord = Coord<int>{x, y} + sh;
                if (!isInImage(coord)) continue;

                if (map[coord.y][coord.x].type != cell::OBSTACLE) {
                    coords.push({x, y});
                    visited[y][x] = true;    
                    break;
                }
            }
        }
    }
}

void ObstacleTransformer::updateRowsAndCols(const CellMap& map) {
    rows = map.size();
    cols = rows > 0 ? map[0].size() : 0;
}