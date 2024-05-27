#pragma once

#include "bitmap_image.hpp"

#include "Coord.hpp"
#include "Types.hpp"

namespace ImageMarker {
    using CellMap = std::vector<std::vector<Cell>>;

    // Mark cells on an image using cell map.
    void markCells(bitmap_image& image, const CellMap& cellMap, int cellSize);

    // Mark path on an image using cell path coordinates and cell size.
    void markPath(bitmap_image& image,
                  const std::vector<Coord<int>>& pathCoords,
                  int cellSize);
};