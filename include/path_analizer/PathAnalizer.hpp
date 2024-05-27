#pragma once

#include <vector>

#include "Coord.hpp"

// Simple path analizer.
// Use path length and overall turn angle to choose the best path.
// Costfunction = length * lengthWeight + angle * angleWeight.
class PathAnalizer {
public:
    PathAnalizer(double lengthWeightArg, double angleWeightArg);

    std::vector<Coord<int>>
    findBestPath(const std::vector<std::vector<Coord<int>>>& paths);

private:
    float computeAngle(const std::vector<Coord<int>>& path) const;

    long long computePathLength(const std::vector<Coord<int>>& path) const;

    inline double getDist(const Coord<int>& c0, const Coord<int>& c1) const;

    struct PathStat {
        int i;
        long long length;
        float angle;
    };

    std::vector<PathStat> pathsStat;

    double lengthWeight = 0.7;
    double angleWeight  = 0.3;
};