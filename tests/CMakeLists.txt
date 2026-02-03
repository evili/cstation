cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_openlcb_can)

target_sources(app PRIVATE src/test_openlcb_can.c
                   ${CMAKE_CURRENT_SOURCE_DIR}/../src/../src/openlcb_core.c
                   ${CMAKE_CURRENT_SOURCE_DIR}/../src/../src/openlcb_can.c)

# Ensure the tests can see project includes
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
