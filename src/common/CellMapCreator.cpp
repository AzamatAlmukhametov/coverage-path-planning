#include "CellMapCreator.hpp"

using CellMap = std::vector<std::vector<Cell>>;

CellMap CellMapCreator::create(const bitmap_image& image, int cellSize) const {
    const int rows = image.height() / cellSize + (image.height() % cellSize == 0 ? 0 : 1);
    const int cols = image.width()  / cellSize + (image.width()  % cellSize == 0 ? 0 : 1);
    const int initWeight = 1;

    CellMap matrix(rows, std::vector<Cell>(cols));
    for (int y = 0, imageY = 0; y < rows; ++y, imageY += cellSize) {
        for (int x = 0, imageX = 0; x < cols; ++x, imageX += cellSize) {
            int occupied = calcOccupiedArea(image, imageX, imageY, cellSize);
            if (occupied == 0) {
                matrix[y][x] = {cell::FREE, initWeight};
            } else if (occupied > occupiedThreshold) {
                matrix[y][x] = {cell::OBSTACLE, initWeight};
            } else {
                matrix[y][x] = {cell::PARTIALLY_FREE, initWeight};
            }
        }
    }

    return matrix;
}

bool CellMapCreator::setOccupiedThreshold(int val) {
    if (val < 0 || 100 > val) {
        return false;
    }

    occupiedThreshold = val;
    return true;
}

void CellMapCreator::setFreeSpaceColour(rgb_t colour) {
    freeSpaceColour = colour;
}

int CellMapCreator::calcOccupiedArea(const bitmap_image& image, int x0, int y0, int cellSize) const {
    int count = 0;

    const int yBorder = (y0 + cellSize >= image.height() ? image.height() : y0 + cellSize);
    const int xBorder = (x0 + cellSize >= image.width()  ? image.width()  : x0 + cellSize);

    for (int y = y0; y < yBorder; ++y) {
        for (int x = x0; x < xBorder; ++x) {
            rgb_t colour;
            image.get_pixel(x, y, colour);
            if (colour != freeSpaceColour) {
                ++count;
            }
        }
    }

    return count * 100 / ((yBorder - y0) * (xBorder - x0));
}