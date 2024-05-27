#include "CoordMapper.hpp"

#include <stdexcept>

Coord<int> CoordMapper::map(const Coord<int>& src, int srcCellSize, int dstCellSize) {
    Coord<int> dst;
    dst.x = (src.x * srcCellSize + srcCellSize / 2) / dstCellSize;
    dst.y = (src.y * srcCellSize + srcCellSize / 2) / dstCellSize;
    return dst;
}

std::vector<Coord<int>>
CoordMapper::mapCellMapToImg(const std::vector<Coord<int>>& src,
                                   int scrCellSize, int width, int height) {
    std::vector<Coord<int>> dst;
    for (auto coord : src) {
        coord.x *= scrCellSize;
        coord.y *= scrCellSize;
        if (coord.x >= width || coord.y >= height) {
            throw std::runtime_error("coordinates out of range");
        }

        coord.x += (coord.x + scrCellSize < width)  ? scrCellSize / 2 : (width  - coord.x) / 2;
        coord.y += (coord.y + scrCellSize < height) ? scrCellSize / 2 : (height - coord.y) / 2;
        dst.push_back(coord);
    }

    return dst;
}

std::vector<Coord<int>>
CoordMapper::mapImgToCellMap(const std::vector<Coord<int>>& src,
                                   int dstCellSize, int width, int height) {
    std::vector<Coord<int>> dst;
    for (auto coord : src) {
        coord.x = coord.x / dstCellSize;
        coord.y = coord.y / dstCellSize;
        if (coord.x >= width || coord.y >= height) {
            throw std::runtime_error("coordinates out of range");
        }
        dst.push_back(coord);
    }

    return dst;
}