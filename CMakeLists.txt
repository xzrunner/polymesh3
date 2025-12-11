cmake_minimum_required(VERSION 3.16)

project(polymesh3)

add_definitions(
    -DNO_BOOST
)

################################################################################
# Source groups
################################################################################
set(dataset
    "include/polymesh3/Polytope.h"
    "include/polymesh3/TexCoord.h"
    "source/Polytope.cpp"
)
source_group("dataset" FILES ${dataset})

set(tools
    "include/polymesh3/MeshBuilder.h"
    "include/polymesh3/PolytopeAlgos.h"
    "include/polymesh3/TopoPolyAdapter.h"
    "source/MeshBuilder.cpp"
    "source/PolytopeAlgos.cpp"
    "source/TopoPolyAdapter.cpp"
)
source_group("tools" FILES ${tools})

set(ALL_FILES
    ${dataset}
    ${tools}
)

add_library(${PROJECT_NAME} STATIC ${ALL_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE include external/sm external/halfedge/include)
