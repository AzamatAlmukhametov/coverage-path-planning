#include <fstream>
#include <iostream>

#include "bitmap_image.hpp"

#include "CellMapCreator.hpp"
#include "ColourDefinitions.hpp"
#include "Coord.hpp"
#include "CoordMapper.hpp"
#include "GlobalPathPlanner.hpp"
#include "ImageMarker.hpp"
#include "PathTransformer.hpp"
#include "Types.hpp"

namespace configs {
    const std::string inputMapFileName = "./output/inflated.bmp";
    const std::string outputMapFileName = "./output/routeMap.bmp";
    const std::string outputGlobalPathCoordsFileName = "./output/globalPathCoords.txt";
    const int cellSize = 50;
    const int occupiedThreshold = 80; // 0..100%
    const Coord<int> startCoord = {1, 1};
    const int otMulti = 1;
    const int dtMulti = 1;
};

int main()
{
    std::cout << "Read inflated map" << std::endl;
    bitmap_image image(configs::inputMapFileName);

    if (!image) {
        std::cerr << "Error - Failed to open: "
                  << configs::inputMapFileName << std::endl;
        return 1;
    }

    std::cout << "Create cell map" << std::endl;
    CellMapCreator omc;
    omc.setOccupiedThreshold(configs::occupiedThreshold);
    omc.setFreeSpaceColour(freeSpaceColour);
    auto cellMap = omc.create(image, configs::cellSize);

    std::cout << "Apply Path transformer" << std::endl;
    PathTransformer pt(cellMap, configs::startCoord);
    pt.setObstacleTransformerMultiplier(configs::otMulti);
    pt.setDistanceTransformerMultiplier(configs::dtMulti);
    auto weightedMap = pt.transform();

    std::cout << "Plan Global path" << std::endl;
    GlobalPathPlanner gpp;
    auto globalPath = gpp.planPath(weightedMap, configs::startCoord);
    if (globalPath.empty()) {
        std::cout << "No global path coordinates are generated" << std::endl;
        return 0;
    }

    std::cout << "Output global path coords" << std::endl;
    std::ofstream globalPathCoords(configs::outputGlobalPathCoordsFileName);
    auto globalPathMapped =
        CoordMapper::mapCellMapToImg(globalPath, configs::cellSize,
                                     image.width(), image.height());
    for (auto coord : globalPathMapped) {
        globalPathCoords << coord.x << ' ' << coord.y << std::endl;
    }

    std::cout << "Mark cells" << std::endl;
    ImageMarker::markCells(image, cellMap, configs::cellSize);

    std::cout << "Mark path" << std::endl;
    ImageMarker::markPath(image, globalPath, configs::cellSize);

    std::cout << "Save image" << std::endl;
    image.save_image(configs::outputMapFileName);

    return 0;
}
