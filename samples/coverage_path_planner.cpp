#include <stdexcept>
	
#include <fstream>
#include <iostream>
#include <vector>

#include "bitmap_image.hpp"

#include "CellMapCreator.hpp"
#include "ColourDefinitions.hpp"
#include "Coord.hpp"
#include "CoordMapper.hpp"
#include "GlobalPathPlanner.hpp"
#include "ImageMarker.hpp"
#include "Inflator.hpp"
#include "LocalPathPlanner.hpp"
#include "PathAnalizer.hpp"
#include "PathTransformer.hpp"
#include "Types.hpp"

namespace configs {
    const std::string inputMapFileName = "./input/map.bmp";
    const std::string inputInflatedMapFileName = "./output/inflated.bmp";
    const std::string outputMapFileName = "./output/coveragePathMap.bmp";
    const std::string outputPathCoordsFileName = "./output/coveragePathCoords.txt";

    const Coord<int> startCoordOnImage = {35, 35};

    // Inflator settings.
    const int inflationRadius = 25;   

    // Global planner settings.
    const int globalPathCellSize = 50;
    const int globalPathOccupiedThreshold = 80; // 0..100%

    // Costfunction weights range settings.
    const int otMultiMin = 0, otMultiMax = 40;
    const int dtMultiMin = 0, dtMultiMax = 40;
    const int otDelta = 3, dtDelta = 3;

    // Path analizer settings. Sum of weights must be <= 1
    const double lengthWeight = 0.7;
    const double angleWeight = 0.3;

    // Local planner settings.
    const int localPathCellSize = 5;
    const int localPathOccupiedThreshold = 80; // 0..100%
    const LocalPathPlanner::wavePropAlg lppWavePropAlg = LocalPathPlanner::wavePropAlg::TOWARD_TO_AIM;
};

bitmap_image getInflatedImage() {   
    bitmap_image image(configs::inputMapFileName);
    if (!image) {
        throw std::runtime_error("Error - Failed to open: " + configs::inputMapFileName);
    }

    Inflator inf(image);
    inf.inflate(configs::inflationRadius);

    bitmap_image inflated = inf.getImage();

    std::cout << "saving image" << std::endl;
    inflated.save_image(configs::inputInflatedMapFileName);

    return inflated;
}

std::vector<Coord<int>> planGlobalPath(const bitmap_image& image) {
    const Coord<int> startCoord =
        CoordMapper::map(configs::startCoordOnImage, 1, configs::globalPathCellSize);

    std::cout << "Create cell map" << std::endl;
    CellMapCreator omc;
    omc.setOccupiedThreshold(configs::globalPathOccupiedThreshold);
    omc.setFreeSpaceColour(freeSpaceColour);
    auto cellMap = omc.create(image, configs::globalPathCellSize);

    std::cout << "Generate paths" << std::endl;
    std::vector<std::vector<Coord<int>>> paths;
    PathTransformer pt(cellMap, startCoord);
    for (int dtMulti = configs::dtMultiMin;
             dtMulti <= configs::dtMultiMax; dtMulti += configs::dtDelta) {
        for (int otMulti = configs::otMultiMin;
                 otMulti <= configs::otMultiMax; otMulti += configs::otDelta) {
            pt.setDistanceTransformerMultiplier(dtMulti);
            pt.setObstacleTransformerMultiplier(otMulti);
            auto weightedMap = pt.transform();

            auto globalPathCellCoords = GlobalPathPlanner().planPath(weightedMap, startCoord);
            paths.push_back(globalPathCellCoords);
        }
    }

    std::cout << "Find the best path" << std::endl;
    PathAnalizer pa(configs::lengthWeight, configs::angleWeight);
    auto bestPath = pa.findBestPath(paths);

    std::cout << "Map coordinates into image coordinates" << std::endl;
    auto globalPath =
        CoordMapper::mapCellMapToImg(bestPath, configs::globalPathCellSize,
                                     image.width(), image.height());
    return globalPath;
}

std::vector<Coord<int>> planLocalPath(const bitmap_image& image,
                                      const std::vector<Coord<int>>& globalPath) {
    
    std::cout << "Create cell map" << std::endl;
    CellMapCreator omc;
    omc.setOccupiedThreshold(configs::localPathOccupiedThreshold);
    omc.setFreeSpaceColour(freeSpaceColour);
    auto cellMap = omc.create(image, configs::localPathCellSize);

    std::cout << "Convert global path coords into cell map coordinates" << std::endl;
    int cellMapRows = cellMap.size();
    int cellMapCols = cellMapRows > 0 ? cellMap[0].size() : 0;
    auto globalPathCellCoords =
        CoordMapper::mapImgToCellMap(globalPath, configs::localPathCellSize,
                                     cellMapCols, cellMapRows);

    std::cout << "Plan local path" << std::endl;
    LocalPathPlanner lpp(configs::lppWavePropAlg);
    auto localPathCellCoords = lpp.planPath(cellMap, globalPathCellCoords);

    auto localPath =
        CoordMapper::mapCellMapToImg(localPathCellCoords, configs::localPathCellSize,
                                     image.width(), image.height());
    
    return localPath;
}

bool isFileExists(const std::string& fileName) {
    std::ifstream f(fileName);
    return f.good();
}

int main()
{
    std::cout << "---------------------------------" << std::endl;
    std::cout << "Create infalted image" << std::endl;

    bitmap_image inflatedImage;
    if (isFileExists(configs::inputInflatedMapFileName)) {
        std::cout << "Infalted image already created" << std::endl;
        inflatedImage = bitmap_image(configs::inputInflatedMapFileName);
        if (!inflatedImage) {
            std::cerr << "Error - Failed to open: "
                      << configs::inputInflatedMapFileName << std::endl;
            return 1;
        }
    } else {
        inflatedImage = getInflatedImage();
    }

    std::cout << "successfull" << std::endl;
    std::cout << "---------------------------------" << std::endl;

    std::cout << "---------------------------------" << std::endl;
    std::cout << "Plan Global path" << std::endl;
    auto globalPath = planGlobalPath(inflatedImage);
    if (globalPath.empty()) {
        std::cout << "No global path coordinates are generated" << std::endl;
        return 0;
    }
    std::cout << "successfull" << std::endl;
    std::cout << "---------------------------------" << std::endl;

    std::cout << "---------------------------------" << std::endl;
    std::cout << "Plan Local path" << std::endl;
    auto localPath = planLocalPath(inflatedImage, globalPath);
    std::cout << "successfull" << std::endl;
    std::cout << "---------------------------------" << std::endl;

    ImageMarker::markPath(inflatedImage, localPath, 1);

    inflatedImage.save_image(configs::outputMapFileName);

    std::ofstream pathCoords(configs::outputPathCoordsFileName);
    for (auto coord : localPath) {
        pathCoords << coord.x << ' ' << coord.y << std::endl;
    }

    return 0;
}
