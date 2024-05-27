#include "ImageMarker.hpp"

#include <cassert>
#include <stdexcept>

#include "ColourDefinitions.hpp"

using CellMap = std::vector<std::vector<Cell>>;

void ImageMarker::markCells(bitmap_image& image, const CellMap& cellMap, int cellSize) {
    const int penWidth = 1;
    assert(cellSize >= penWidth * 2);

    image_drawer drawer(image);
    drawer.pen_width(penWidth);

    const int rows = cellMap.size();
    const int cols = rows > 0 ? cellMap[0].size() : 0;

    for (int y = 0, y1 = 0; y < rows; ++y, y1 += cellSize) {
        for (int x = 0, x1 = 0; x < cols; ++x, x1 += cellSize) {
            rgb_t penColour;
            if (cellMap[y][x].type == cell::FREE) {
                penColour = freeRectColour;
            } else if (cellMap[y][x].type == cell::OBSTACLE) {
                penColour = obstacleRectColour;
            } else if (cellMap[y][x].type == cell::PARTIALLY_FREE) {
                penColour = partiallyFreeRectColour;
            } else {
                throw std::runtime_error("unknown cell type");
            }
            drawer.pen_color(penColour);

            int y2 = (y1 + cellSize >= image.height() ? image.height() - 1 : y1 + cellSize) - penWidth;
            int x2 = (x1 + cellSize >= image.width()  ? image.width()  - 1 : x1 + cellSize) - penWidth;
            drawer.rectangle(x1, y1, x2, y2);
        }
    }
}

void ImageMarker::markPath(bitmap_image& image,
                           const std::vector<Coord<int>>& pathCoords,
                           int cellSize) {
    const int penWidth = 1;
    const int radius = 1;
    assert(radius >= penWidth);

    image_drawer circleDrawer(image);
    circleDrawer.pen_width(penWidth);
    circleDrawer.pen_color(pathMarkColour);

    image_drawer pathDrawer(image);
    pathDrawer.pen_width(penWidth);
    pathDrawer.pen_color(pathMarkColour);

    int xPrev = 0, yPrev = 0;
    for (int i = 0; i < pathCoords.size(); ++i) {
        int x = pathCoords[i].x * cellSize + cellSize / 2;
        if (x >= image.width()) {
            x = pathCoords[i].x * cellSize + (image.width() - 1 - pathCoords[i].x * cellSize) / 2;
        }

        int y = pathCoords[i].y * cellSize + cellSize / 2;
        if (y >= image.height()) {
            y = pathCoords[i].y * cellSize + (image.height() - 1 - pathCoords[i].y * cellSize) / 2;
        }

        circleDrawer.circle(x, y, radius);

        if (i > 0) {
            pathDrawer.line_segment(x, y, xPrev, yPrev);
        }
        xPrev = x, yPrev = y;
    }
}
