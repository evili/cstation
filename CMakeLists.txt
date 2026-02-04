cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cstation)


set(SOURCES
  src/main.c
  src/lcc.c
  src/lcc_can.c
)
      
# Expose project include/ to the compiler
target_include_directories(app PRIVATE include)
target_sources(app PRIVATE ${SOURCES} ${HEADERS})
