cmake_minimum_required(VERSION 2.8)

project(coverage_path_planning)

set(CMAKE_CXX_STANDARD 14)

include_directories(
    include/common
    include/global_path_planner
    include/inflator
    include/local_path_planner
    include/path_analizer
)

set(COMMON
    src/common/CellMapCreator.cpp
    src/common/CoordMapper.cpp 
    src/common/ImageMarker.cpp
)

set(GLOBAL_PATH_PLANNER
    src/global_path_planner/DistanceTransformer.cpp 
    src/global_path_planner/GlobalPathPlanner.cpp
    src/global_path_planner/ObstacleTransformer.cpp 
    src/global_path_planner/PathTransformer.cpp
)

set(INFLATOR
    src/inflator/Inflator.cpp
)

set(LOCAL_PATH_PLANNER 
    src/local_path_planner/LocalPathPlanner.cpp
)

set(PATH_ANALIZER 
    src/path_analizer/PathAnalizer.cpp
)

add_executable(
    ${PROJECT_NAME} samples/coverage_path_planner.cpp
    ${COMMON}
    ${GLOBAL_PATH_PLANNER}
    ${INFLATOR}
    ${LOCAL_PATH_PLANNER}
    ${PATH_ANALIZER}
)

add_executable(
    inflator samples/inflator.cpp
    ${COMMON}
    ${INFLATOR}
)

add_executable(
    global_path_planner samples/global_path_planner.cpp
    ${COMMON}
    ${GLOBAL_PATH_PLANNER}
)

add_executable(
    local_path_planner samples/local_path_planner.cpp
    ${COMMON}
    ${LOCAL_PATH_PLANNER}
)

file(COPY samples/input  DESTINATION ${CMAKE_BINARY_DIR})
file(COPY samples/output DESTINATION ${CMAKE_BINARY_DIR})