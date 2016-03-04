PROJECT(envire_maps)
cmake_minimum_required(VERSION 2.6)
find_package(Rock)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(envire_mapsBase)

rock_find_pkgconfig(ENVIRE_MAPS-VIZ envire_maps-viz REQUIRED)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

