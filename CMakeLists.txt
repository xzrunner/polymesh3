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

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(..)
include_directories(../sm)
include_directories(../halfedge/include)

add_library(polymesh3 ${ALL_FILES})
