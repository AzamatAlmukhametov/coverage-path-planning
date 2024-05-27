#include "LocalPathPlanner.hpp"

#include <iostream>
#include <queue>
#include <cmath>
#include <stdexcept>

#include "Shifts.hpp"

using CellMap = std::vector<std::vector<Cell>>;

LocalPathPlanner::LocalPathPlanner(wavePropAlg algArg) : alg(algArg) {
    using namespace std::placeholders;
    if (alg == wavePropAlg::AROUND) {
        propagateWave = std::bind(&LocalPathPlanner::propagateWaveAround, this, _1, _2, _3);
    } else if (alg == wavePropAlg::TOWARD_TO_AIM) {
        propagateWave = std::bind(&LocalPathPlanner::propagateWaveTowardToAim, this, _1, _2, _3);
    } else {
        throw std::runtime_error("LocalPathPlanner::wavePropAlg type is undefined");
    }
}

std::vector<Coord<int>> 
LocalPathPlanner::planPath(const CellMap& cellMap,
                           const std::vector<Coord<int>>& coords) {
    if (coords.empty()) {
        throw std::runtime_error("no coordinates for path planning");
    }

    rows = cellMap.size();
    cols = rows > 0 ? cellMap[0].size() : 0;

    if (rows == 0 && cols == 0) {
        throw std::runtime_error("zero size cell map");
    }

    if (!isInImage(coords[0])) {
        throw std::runtime_error("start coordinate is out of range");
    }

    map = std::vector<std::vector<Node>>(rows, std::vector<Node>(cols));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            map[i][j] = {cellMap[i][j], initOperationId};
        }
    }

    Coord<int> start = coords[0];
    if (cellMap[start.y][start.x].type == cell::OBSTACLE &&
        !findNearNotObstCell(start)) {
        throw std::runtime_error("start coordinate is invalid");
    }

    std::vector<Coord<int>> path{start};
    for (int i = 1; i < coords.size(); ++i) {
        start = path.back();
        Coord<int> end = coords[i];

        if (!isInImage(end)) {
            throw std::runtime_error("coordinate is out of range");
        }
    
        if (map[end.y][end.x].cell.type == cell::OBSTACLE &&
            !findNearNotObstCell(end)) {
            continue;
        }

        addToPath(start, end, path, i);
    }

    return path;
}

bool LocalPathPlanner::isInImage(const Coord<int>& coord) const {
    return coord.y >= 0 && coord.y < rows &&
           coord.x >= 0 && coord.x < cols;
}

bool LocalPathPlanner::findNearNotObstCell(Coord<int>& base) {
    std::queue<Coord<int>> q;
    q.push(base);

    while (!q.empty()) {
        int size = q.size();
        while (size--) {
            auto front = q.front();
            q.pop();

            for (auto sh : shift4) {
                Coord<int> coord = front + sh;

                if (!isInImage(coord)) continue;
                if (map[coord.y][coord.x].operationId == findNearNotObstCellOperationId) continue;

                map[coord.y][coord.x].operationId = findNearNotObstCellOperationId;

                int dist = round(sqrt((pow(coord.x - (double)base.x, 2) +
                                       pow(coord.y - (double)base.y, 2))));
                
                if (dist > searchAreaRadius) continue;

                if (map[coord.y][coord.x].cell.type != cell::OBSTACLE) {
                    base = coord;
                    return true;
                }
                
                q.push(coord);
            }
        }
    }

    return false;
}

bool LocalPathPlanner::propagateWaveAround(const Coord<int>& start,
                                           const Coord<int>& end,
                                                 int operationId) {
    std::queue<Coord<int>> q;
    q.push(end);

    map[end.y][end.x].cell.weight = 0;
    map[end.y][end.x].operationId = operationId;

    while (!q.empty()) {
        int size = q.size();
        while (size--) {
            auto base = q.front();
            q.pop();
            if (base == start) {
                return true;
            }

            for (auto sh : shift4) {
                Coord<int> coord = base + sh;

                if (!isInImage(coord)) continue;
                if (map[coord.y][coord.x].cell.type == cell::OBSTACLE) continue;
                if (map[coord.y][coord.x].operationId == operationId) continue;

                map[coord.y][coord.x].operationId = operationId;
                map[coord.y][coord.x].cell.weight =
                    map[base.y][base.x].cell.weight + 1;
                
                q.push(coord);
            }
        }
    }

    return false;
}

bool LocalPathPlanner::propagateWaveTowardToAim(const Coord<int>& start,
                                                const Coord<int>& end,
                                                      int operationId) {
    auto cmpDistToStart = [&start] (const Coord<int>& lhs, const Coord<int>& rhs) {
        int lhsDist = round(sqrt((pow(start.x - (double)lhs.x, 2) +
                                  pow(start.y - (double)lhs.y, 2))));
        int rhsDist = round(sqrt((pow(start.x - (double)rhs.x, 2) +
                                  pow(start.y - (double)rhs.y, 2))));
        return lhsDist > rhsDist;                                  
    };

    std::priority_queue<Coord<int>, std::vector<Coord<int>>, decltype(cmpDistToStart)> pq(cmpDistToStart);
    pq.push(end);

    map[end.y][end.x].cell.weight = 0;
    map[end.y][end.x].operationId = operationId;

    while (!pq.empty()) {
        auto base = pq.top();
        pq.pop();
        if (base == start) {
            return true;
        }

        for (auto sh : shift4) {
            Coord<int> coord = base + sh;

            if (!isInImage(coord)) continue;
            if (map[coord.y][coord.x].cell.type == cell::OBSTACLE) continue;
            if (map[coord.y][coord.x].operationId == operationId) continue;

            map[coord.y][coord.x].operationId = operationId;
            map[coord.y][coord.x].cell.weight =
                map[base.y][base.x].cell.weight + 1;
            
            pq.push(coord);
        }
    }

    return false;
}

void LocalPathPlanner::addToPath(const Coord<int>& start, const Coord<int>& end,
                                       std::vector<Coord<int>>& path, int operationId) {
    if (!propagateWave(start, end, operationId)) {
        std::cout << "wave doesn't reach the aim" << std::endl;
        return;
    }

    Coord<int> curr = start;
    while (curr != end) {
        path.push_back(curr);

        Coord<int> minWeightCoord = curr;
        for (auto sh : shift8) {
            Coord<int> coord = curr + sh;

            if (!isInImage(coord)) continue;
            if (map[coord.y][coord.x].cell.type == cell::OBSTACLE) continue;
            if (map[coord.y][coord.x].operationId != operationId) continue;

            int minWeight = map[minWeightCoord.y][minWeightCoord.x].cell.weight;
            int weight = map[coord.y][coord.x].cell.weight;
            if (minWeight > weight) {
                minWeightCoord = coord;
            }
        }

        if (curr == minWeightCoord) {
            std::cout << "path not found" << std::endl;
            return;
        }
        curr = minWeightCoord;
    }
    path.push_back(curr);
}