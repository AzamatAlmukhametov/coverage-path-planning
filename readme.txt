    Additional libs (http://www.partow.net/programming/bitmap/index.html):
C++ Bitmap Library (bitmap_image.hpp). Lib path in project: include/common/bitmap_image.hpp

    This work based on article by A. Zelinsky1, R.A. Jarvis2, J.C. Byrne2 and 
S. Yuta3 "Planning Paths of Complete Coverage of an Unstructured Environment 
by a Mobile Robot"

    The program is a draft. Coverage path planning implementation use cell decomposition
of image, Obstacle transformation, Distance transformation, wave algorithm, cost function.
The approach consider the initial map as a static (not changing environment).

Process steps:
1 Inflate obstacles on the initial map.
2 Create cell map; Apply Path transformer; Plan Global path.
3 Create cell map; Plan Local path.
(4 Mark cells, Mark path, Save image.)

    Inflator.
Puppose: forming safe distance around obstacle.
Brute force algorithm for inflation obstacle borders is slow. A new approuch was
used to speed up the obstacle inflation, but it has some flaws (shapes are not purfect).
Inflation process is performed by subinflation steps. If an end radius of inflation
is set, then in each step obstacle inflated to the part (ri + dr) of the whole radius(r).
Each step dr is is changind (ddr). At each step a goal radius ri + dr is calculeted.
    r0 = 0;
    r1 = r0 + dr; dr = dr + ddr;
    r2 = r1 + dr; dr = dr + ddr; ...
To fill the area ri + dr around obstacle wave algorithm is used. The starting
coordinates of the wave include:
    obstacle border;
    coordinates of a line segment that starts at the obstacle border and with the
length of ri + dr. The direction of the line segment is defined by shift8 matrix
(see in code).

    Global path planner.
Purpose: planning visiting cells order.
After reading inflated map from input file, the map converting into cell map. Each cell
represent the occupied/non-occupied status. The Distance and Obstacle transformations are
appying next to cell map creating two weighted maps. The weights depends on the
transformation types (see in article). Then using two weighted maps and cost function
(see in article) with scaler a new weight map is creating. Global planner use the weighted map to
plan map coverage path. At each step global planner choose the cell with least weight.
If there is no free cells nearby, then the previous positions are checked.

    Local path planner.
Purpose: finding path between global path coordinates.
Based on wave propogation algorithms (toward to aim, around), weight map. With given
cell map and sequence of coordinates the Local path planner tries to connect coordinates
(cells) in given order. The connection is supposed to form a the shortest path between
two coordinates (cells). Sequentially all coordinates connected forming path.

Configs:
    general:
        Initial\output map file path;
        Output coordinates file path;
        Start coordinate on image;

    inflator:
        inflation radius - radius of safe distance around obstacle;

    global path planner:
        cell size - size of the cell global planner plan to visit (influences to
            distance between paths);
        occupied threshold - cell evaluated as an obstacel
            if numbers of occupied pixel more that this value; (0..100%)
        obstacle transformer multiplier - weight multiplier for obstacle transformer map;
        distance transformer multiplier - weight multiplier for distance transformer map;
            const double scaler = dtMaxWeight == 0 ? 1 : (double)otMaxWeight / dtMaxWeight;
            map[y][x].weight = (int)round(scaler * (dtMulti * dtMap[y][x].weight)) +
                                otMulti * otMaxWeight / otMap[y][x].weight;

    local path planner:
        cell size - size of the cell to find path between global coordinates;
        occupied threshold (0..100%);
        wave propogation algorithm:
            toward to aim - wave tend to propogate toward to aim;
            around - wave tend to propogate around (not formig round) initial position;

Configure samples:
    All settings under namespace "configs".
    After changing settings sample need to be rebuild.

Building samples:
    cd coverage-path-planning
    mkdir build
    cd build
    cmake ..

    build all:
    make

    build specific:
    make coverage_path_planning
    make inflator
    make global_path_planner
    make local_path_planner

Run samples:
    ./coverage_path_planning
    ./inflator
    ./global_path_planner
    ./local_path_planner

Default output results folder is "output".