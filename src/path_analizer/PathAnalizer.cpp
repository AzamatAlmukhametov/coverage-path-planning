#include "PathAnalizer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

PathAnalizer::PathAnalizer(double lengthWeightArg, double angleWeightArg) {
    if (lengthWeightArg < 0 || angleWeightArg < 0) {
        throw std::runtime_error("weights must be positive");
    }

    if (lengthWeightArg + angleWeightArg - 1 > 0.1) {
        throw std::runtime_error("weights sum must be <= 1");
    }

    lengthWeight = lengthWeightArg;
    angleWeight = angleWeightArg;
}

std::vector<Coord<int>>
PathAnalizer::findBestPath(const std::vector<std::vector<Coord<int>>>& paths) {
    if (paths.empty()) {
        std::cout << "there is no paths no analize" << std::endl;
        return {};
    }
    std::cout << paths.size() << " paths will be analized" << std::endl;

    pathsStat.resize(paths.size());

    for (int i = 0; i < paths.size(); ++i) {
        long long pathLength = computePathLength(paths[i]);
        float angle = computeAngle(paths[i]);
        pathsStat[i] = {i, pathLength, angle};
    }

    auto cmp = [lengthWeight=lengthWeight, angleWeight=angleWeight]
               (const PathStat& lhs, const PathStat& rhs) {
        return (lhs.length - rhs.length) * lengthWeight +
               (lhs.angle  - rhs.angle)  * angleWeight  < 0;
    };

    std::sort(pathsStat.begin(), pathsStat.end(), cmp);

    return paths[pathsStat[0].i];
}

float PathAnalizer::computeAngle(const std::vector<Coord<int>>& path) const {
    if (path.size() < 3) {
        return 0;
    }

    float angle = 0;
    for (int i = 1; i < path.size() - 1; ++i) {
        if (path[i + 1].x - path[i].x == 0) {
            angle += M_PI / 2;
        } else {
            angle += std::abs(std::atan(float(path[i + 1].y - path[i].y) /
                                        float(path[i + 1].x - path[i].x)));
        }
    }

    return angle;
}

long long PathAnalizer::computePathLength(const std::vector<Coord<int>>& path) const {
    double length = 0;
    for (int i = 0; i < path.size() - 1; ++i) {
        length += getDist(path[i], path[i + 1]);
    }
    return lround(length);
}

inline double PathAnalizer::getDist(const Coord<int>& c0, const Coord<int>& c1) const {
    return sqrt(pow(c0.x - c1.x, 2) + pow(c0.y - c1.y, 2));
}
