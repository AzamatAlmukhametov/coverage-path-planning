#include "GlobalPathPlanner.hpp"

#include <algorithm>
#include <stdexcept>

#include <stack>

#include "Shifts.hpp"

using CellMap = std::vector<std::vector<Cell>>;

std::vector<Coord<int>>
GlobalPathPlanner::planPath(const CellMap& weightMap,
                            const Coord<int>& start) {
    updateRowsAndCols(weightMap);
    if (!setStartCoord(weightMap, start)) {
        throw std::runtime_error("start coordinate is invalid");
    }

    std::vector<Coord<int>> path;
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols));

    std::stack<std::vector<Node>> s;
    s.push({{start, weightMap[start.y][start.x]}});

    while (!s.empty()) {
        auto& nodes = s.top();
        if (nodes.empty()) {
            s.pop();
            continue;
        }

        Node node = nodes.back();
        nodes.pop_back();
        if (visited[node.coord.y][node.coord.x]) continue;

        std::vector<Node> newNodes;
        for (auto sh : shift8) {
            Coord<int> coord = node.coord + sh;

            if (!isInImage(coord)) continue;
            if (weightMap[coord.y][coord.x].type == cell::OBSTACLE) continue;
            if (visited[coord.y][coord.x]) continue;

            newNodes.push_back({coord, weightMap[coord.y][coord.x]});
        }

        visited[node.coord.y][node.coord.x] = true;
        path.push_back(node.coord);

        if (newNodes.empty()) continue;
        std::sort(newNodes.begin(), newNodes.end(), std::greater<Node>());
        s.push(newNodes);
    }

    return path;
}

bool GlobalPathPlanner::isInImage(const Coord<int>& coord) const {
    return coord.y >= 0 && coord.y < rows &&
           coord.x >= 0 && coord.x < cols;
}

void GlobalPathPlanner::updateRowsAndCols(const CellMap& weightMap) {
    rows = weightMap.size();
    cols = rows > 0 ? weightMap[0].size() : 0;
}

bool GlobalPathPlanner::setStartCoord(const CellMap& weightMap,
                                      const Coord<int>& startArg) {
    if (isInImage(startArg) &&
        weightMap[startArg.y][startArg.x].type != cell::OBSTACLE) {
        start = startArg;
        return true;
    }

    return false;
}
