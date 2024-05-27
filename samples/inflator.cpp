#include <iostream>

#include "bitmap_image.hpp"

#include "Inflator.hpp"

namespace configs {
    const std::string inputMapFileName = "./input/map.bmp";
    const std::string outputMapFileName = "./output/inflated.bmp";
    const int radius = 25;
};

int main()
{   
    bitmap_image image(configs::inputMapFileName);
    if (!image) {
        std::cerr << "Error - Failed to open: "
                  << configs::inputMapFileName << std::endl;
        return 1;
    }

    Inflator inf(image);
    inf.inflate(configs::radius);

    std::cout << "saving image" << std::endl;
    inf.getImage().save_image(configs::outputMapFileName);

    return 0;
}