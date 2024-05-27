#include <iostream>
#include <fstream>
#include <vector>

#include "bitmap_image.hpp"

#include "CellMapCreator.hpp"
#include "ColourDefinitions.hpp"
#include "Coord.hpp"
#include "CoordMapper.hpp"
#include "ImageMarker.hpp"
#include "LocalPathPlanner.hpp"
#include "Types.hpp"

namespace configs {
    const std::string inputMapFileName = "./output/inflated.bmp";
    const std::string inputGlobalPathCoordsFileName = "./output/globalPathCoords.txt";
    const std::string outputMapFileName = "./output/pathMap.bmp";
    const std::string outputLocalPathCoordsFileName = "./output/localPathCoords.txt";
    const int cellSize = 5;
    const int occupiedThreshold = 80; // 0..100%
    const LocalPathPlanner::wavePropAlg lppWavePropAlg = LocalPathPlanner::wavePropAlg::TOWARD_TO_AIM;
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

    std::cout << "Read global path coords" << std::endl;
    std::ifstream globalPathCoords(configs::inputGlobalPathCoordsFileName);
    std::vector<Coord<int>> globalPath;
    Coord<int> buf;
    while (globalPathCoords >> buf.x >> buf.y) {
        globalPath.push_back(buf);
    }

    std::cout << "Convert global path coords into cell map coordinates" << std::endl;
    int cellMapRows = cellMap.size();
    int cellMapCols = cellMapRows > 0 ? cellMap[0].size() : 0;
    auto globalPathCellCoords =
        CoordMapper::mapImgToCellMap(globalPath, configs::cellSize,
                                     cellMapCols, cellMapRows);

    std::cout << "Plan local path" << std::endl;
    LocalPathPlanner lpp(configs::lppWavePropAlg);
    auto localPath = lpp.planPath(cellMap, globalPathCellCoords);

    std::cout << "Output local path coords" << std::endl;
    std::ofstream localPathCoords(configs::outputLocalPathCoordsFileName);
    auto localPathMapped =
        CoordMapper::mapCellMapToImg(localPath, configs::cellSize,
                                     image.width(), image.height());
    for (auto coord : localPathMapped) {
        localPathCoords << coord.x << ' ' << coord.y << std::endl;
    }

    std::cout << "Mark cells" << std::endl;
    ImageMarker::markCells(image, cellMap, configs::cellSize);

    std::cout << "Mark path" << std::endl;
    ImageMarker::markPath(image, localPath, configs::cellSize);

    std::cout << "Save image" << std::endl;
    image.save_image(configs::outputMapFileName);

    return 0;
}
