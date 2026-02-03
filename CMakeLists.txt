cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cstation)

# Expose project include/ to the compiler so <openlcb.h> is resolvable
target_include_directories(app PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(app PRIVATE ${CMAKE_SOURCE_DIR}/src)