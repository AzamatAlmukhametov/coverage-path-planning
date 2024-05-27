#pragma once

#include <queue>
#include <vector>

#include "bitmap_image.hpp"

#include "Coord.hpp"

class Inflator {
public:
    Inflator(const bitmap_image& img);

    void inflate(int radius);

    bitmap_image getImage() const;

private:
    struct Wave {
        Coord<int> coord;
        int i;
    };
    
    void applyBinFilter();

    bool isInImage(int x, int y) const;

    std::vector<Coord<int>> getObstacleBorder() const;
    
    void filterInflatedCoords(std::vector<Coord<int>>& src,
                              std::vector<Coord<int>>& dst) const;

    void addLineCoords(int x0, int y0, int dx, int dy, int l, int group,
                       std::queue<Wave>& wave) const;

    void addSeedsCoords(const std::vector<Coord<int>>& bases, int radius,
                              std::queue<Wave>& wave) const;

    void inflateFromBase(const std::vector<Coord<int>>& bases, int radius,
                               std::vector<Coord<int>>& newBases);

    bitmap_image image;
    const uint frameWidth = 1;
};