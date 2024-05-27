#pragma once

#include <vector>

#include "Coord.hpp"

const std::vector<Coord<int>> shift4{ { 0, -1 },
                                      { 0,  1 },
                                      {-1,  0 },
                                      { 1,  0 } };

const std::vector<Coord<int>> shift8{ {-1, -1 },
                                      { 1, -1 },
                                      {-1,  1 },
                                      { 1,  1 },
                                      { 0, -1 },
                                      {-1,  0 },
                                      { 1,  0 },
                                      { 0,  1 } };