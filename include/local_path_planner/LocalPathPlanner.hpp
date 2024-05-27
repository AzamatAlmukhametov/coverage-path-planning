#pragma once

#include <vector>
#include <functional>

#include "Coord.hpp"
#include "Types.hpp"

// Plan the shortest path between two points of sequence of points.
// The approach use wave propagation algorithm.
class LocalPathPlanner {
public:
    enum class wavePropAlg {
        AROUND,
        TOWARD_TO_AIM
    };

    using CellMap = std::vector<std::vector<Cell>>;

    LocalPathPlanner(wavePropAlg algArg);

    std::vector<Coord<int>> 
    planPath(const CellMap& cellMap, const std::vector<Coord<int>>& coords);

private:
    bool isInImage(const Coord<int>& coord) const;

    bool findNearNotObstCell(Coord<int>& base);

    bool propagateWaveAround(const Coord<int>& start, const Coord<int>& end, int operationId);
    bool propagateWaveTowardToAim(const Coord<int>& start, const Coord<int>& end, int operationId);

    void addToPath(const Coord<int>& start, const Coord<int>& end,
                         std::vector<Coord<int>>& path, int operationId);

    struct Node {
        Cell cell;
        int operationId = 0;
    };

    wavePropAlg alg = wavePropAlg::AROUND;
    std::function<bool(const Coord<int>&, const Coord<int>&, int)> propagateWave;

    std::vector<std::vector<Node>> map;
    int rows = 0, cols = 0;
    const int initOperationId = -1;

    const int searchAreaRadius = 4; // Radius of search area to find non-obstacle cell
    const int findNearNotObstCellOperationId = -2;
};