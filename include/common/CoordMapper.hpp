#pragma once

#include <vector>

#include "Coord.hpp"

namespace CoordMapper {

    // Map cell map coordinate to another cell map coordinte.
    Coord<int> map(const Coord<int>& src, int srcCellSize, int dstCellSize);

    // Map cell map coordinates to image coordinates.
    std::vector<Coord<int>>
    mapCellMapToImg(const std::vector<Coord<int>>& src, int cellSize, int width, int height);

    // Map image coordinates to cell map coordinates.
    std::vector<Coord<int>>
    mapImgToCellMap(const std::vector<Coord<int>>& src, int cellSize, int width, int height);
}