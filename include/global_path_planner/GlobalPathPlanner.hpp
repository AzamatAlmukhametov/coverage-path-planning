#pragma once

#include <vector>

#include "Coord.hpp"
#include "Types.hpp"

// Plan path using weight map and start coordinate.
// If weight map is the representation of potential field in plane,
// then planned path lies at the bottom of the potential field.
// At each step the next cell with least weight is choosen among neighbour cells.
class GlobalPathPlanner {
public:
    using CellMap = std::vector<std::vector<Cell>>;

    std::vector<Coord<int>> planPath(const CellMap& weightMap,
                                     const Coord<int>& start);

private:
    struct Node {
        Coord<int> coord;
        Cell cell;

        bool operator>(const Node& n) const {
            return cell.weight > n.cell.weight;
        }
    };

    bool isInImage(const Coord<int>& coord) const;

    void updateRowsAndCols(const CellMap& weightMap);

    bool setStartCoord(const CellMap& weightMap, const Coord<int>& startArg);

    int rows = 0, cols = 0;
    Coord<int> start = {0, 0};
};