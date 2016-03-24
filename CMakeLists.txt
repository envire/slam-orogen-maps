PROJECT(maps)
cmake_minimum_required(VERSION 2.6)
find_package(Rock)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(mapsBase)

rock_find_pkgconfig(MAPS-VIZ maps-viz REQUIRED)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

