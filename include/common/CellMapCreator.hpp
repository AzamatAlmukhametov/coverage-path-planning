#pragma once

#include <vector>

#include "bitmap_image.hpp"

#include "Types.hpp"

// Create cell map from image.
// Non free space colour pixels related as an obstacle.
// Parameters:
//   cellSize - size of the the square cell
//   occupiedThreshold - 0..100% - number of non free space pixel color divided by bumbers of pixels in one cell
//   freeSpaceColour - free space image pixel colour
class CellMapCreator {
public:
    using CellMap = std::vector<std::vector<Cell>>;

    CellMap create(const bitmap_image& image, int cellSize) const;

    bool setOccupiedThreshold(int val);

    void setFreeSpaceColour(rgb_t colour);

private:
    int calcOccupiedArea(const bitmap_image& image, int x0, int y0, int cellSize) const;

    int occupiedThreshold = 80;              // 0 .. 100%;
    rgb_t freeSpaceColour = {255, 255, 255}; // default is white colour
};
