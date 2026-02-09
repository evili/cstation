cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cstation)


set(SOURCES
  src/main.cpp
  src/lcc.cpp
  src/lcc_can.cpp
)

set(HEADERS
	include/lcc.hxx
	include/lcc_can.hxx
)

# Expose project include/ to the compiler
target_include_directories(app PRIVATE include)
target_sources(app PRIVATE ${SOURCES} ${HEADERS})
