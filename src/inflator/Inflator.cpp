#include "Inflator.hpp"

#include <iostream>
#include <cmath>

#include "Shifts.hpp"
#include "ColourDefinitions.hpp"

Inflator::Inflator(const bitmap_image& img) {
    image.setwidth_height(img.width() + frameWidth * 2, img.height() + frameWidth * 2);
    if (!image.copy_from(img, frameWidth, frameWidth)) {
        throw std::runtime_error("inflator init error: copy from failed");
    }

    applyBinFilter();
}

void Inflator::inflate(int radius) {
    const int maxSteps = 10;
    const int ddrMin = 3;

    int ddr = 2 * radius / (maxSteps * maxSteps);
    ddr = ddr < ddrMin ? ddrMin : ddr;

    int dr = 3;
    int r = 0;

    std::vector<Coord<int>> bases = getObstacleBorder();
    std::vector<Coord<int>> newBases;

    std::cout << "inflate from: " << r << " to: " << radius << std::endl;
    std::cout << "------------------------------"           << std::endl;

    while (r + dr <= radius) {
        std::cout << "inflating from: " << r << " to: " << r + dr << std::endl;
        inflateFromBase(bases, dr, newBases);
        filterInflatedCoords(newBases, bases);
        r += dr;
        dr += ddr;
    }

    if (r + dr > radius) {
        std::cout << "inflating from: " << r << " to: " << radius << std::endl;
        inflateFromBase(bases, radius - r, newBases);
    }
}

bitmap_image Inflator::getImage() const {
    bitmap_image imageToReturn;
    image.region(frameWidth, frameWidth,
                 image.width() - frameWidth * 2,
                 image.height() - frameWidth * 2, imageToReturn);
    return imageToReturn;
}

void Inflator::applyBinFilter() {
    // not freeSpaceColour pixel colour -> obstColour
    std::cout << "applying binary filter" << std::endl;
    for (int y = frameWidth; y < image.height() - frameWidth; ++y) {
        for (int x = frameWidth; x < image.width() - frameWidth; ++x) {
            rgb_t colour;
            image.get_pixel(x, y, colour);
            if (colour != freeSpaceColour) {
                image.set_pixel(x, y, obstColour);
            }
        }
    }
}

bool Inflator::isInImage(int x, int y) const {
    return x >= frameWidth && x < image.width()  - frameWidth &&
           y >= frameWidth && y < image.height() - frameWidth;
}

std::vector<Coord<int>> Inflator::getObstacleBorder() const {
    std::cout << "getting obstacle border coordinates" << std::endl;

    std::vector<Coord<int>> border;
    for (int y = frameWidth; y < image.height() - frameWidth; ++y) {
        for (int x = frameWidth; x < image.width() - frameWidth; ++x) {
            rgb_t colour;
            image.get_pixel(x, y, colour);
            if (colour == obstColour) {
                for (auto sh : shift4) {
                    rgb_t colour;
                    Coord<int> coord = Coord<int>{x, y} + sh;
                    image.get_pixel(coord.x, coord.y, colour);

                    if (colour == freeSpaceColour) {
                        border.push_back({x, y});
                        break;
                    }
                }
            }
        }
    }

    return border;
}

void Inflator::filterInflatedCoords(std::vector<Coord<int>>& src,
                                    std::vector<Coord<int>>& dst) const {
    dst.clear();

    for (auto coord : src) {
        rgb_t colour;
        image.get_pixel(coord.x, coord.y, colour);
        if (colour == infColour) {
            for (auto sh : shift4) {
                rgb_t colour;
                Coord<int> newCoord = coord + sh;
                image.get_pixel(newCoord.x, newCoord.y, colour);

                if (colour == freeSpaceColour) {
                    dst.push_back(coord);
                    break;
                }
            }
        }
    }
}

void Inflator::addLineCoords(int x0, int y0, int dx, int dy, int l, int group,
                             std::queue<Wave>& wave) const {
    int x = x0 + dx, y = y0 + dy, dl = sqrt(pow(x - x0, 2) + pow(y - y0, 2));
    while (dl < l) {
        if (!isInImage(x, y)) break;

        rgb_t colour;
        image.get_pixel(x, y, colour);
        if (colour == freeSpaceColour) {
            wave.push({{x, y}, group});
        }

        x += dx, y += dy;
        dl = sqrt(pow(x - x0, 2) + pow(y - y0, 2));
    }
}

void Inflator::addSeedsCoords(const std::vector<Coord<int>>& bases, int radius,
                                    std::queue<Wave>& wave) const {
    for (int i = 0; i < bases.size(); ++i) {
        Coord<int> base = bases[i];
        wave.push({base, i});
        for (auto sh : shift8) {
            rgb_t colour;
            Coord<int> coord = base + sh;
            image.get_pixel(coord.x, coord.y, colour);
            if (colour == infColour || colour == obstColour) {
                addLineCoords(base.x, base.y, -sh.x, -sh.y, radius, i, wave);
            }
        }
    }
}

void Inflator::inflateFromBase(const std::vector<Coord<int>>& bases, int radius,
                                     std::vector<Coord<int>>& newBases) {      
    std::queue<Wave> wave;
    addSeedsCoords(bases, radius, wave);

    newBases.clear();

    while (!wave.empty()) {
        size_t size = wave.size();
        while (size--) {
            auto baseWave = wave.front();
            wave.pop();
            for (auto sh : shift8) {
                rgb_t colour;
                Coord<int> newCoord = baseWave.coord + sh;
                Coord<int> origin = bases[baseWave.i];

                int dist = round(sqrt((pow(newCoord.x - (double)origin.x, 2) +
                                       pow(newCoord.y - (double)origin.y, 2))));
                
                if (dist > radius) continue;

                image.get_pixel(newCoord.x, newCoord.y, colour);

                if (colour == freeSpaceColour) {
                    wave.push({newCoord, baseWave.i});
                    image.set_pixel(newCoord.x, newCoord.y, infColour);
                    newBases.push_back({newCoord});
                }
            }
        }
    }
}