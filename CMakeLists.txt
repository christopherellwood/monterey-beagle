cmake_minimum_required(VERSION 2.6)
project(monterey-beagle)

SET(GCC_COVERAGE_COMPILE_FLAGS "-pthread -lpthread")
SET(CMAKE_SHARED_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")

add_definitions(-D_CRT_SECURE_NO_WARNINGS)


FILE(GLOB SRC "src/*.cpp")
FILE(GLOB HDR "src/*.h")

add_executable(monterey-beagle ${SRC} ${HDR})

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS}" )

target_link_libraries(monterey-beagle)
