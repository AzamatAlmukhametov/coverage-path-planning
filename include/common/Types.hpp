#pragma once

enum class cell {
    FREE,
    OBSTACLE,
    PARTIALLY_FREE
};

struct Cell {
    cell type;
    int weight;
};
