# Required by CMake 3.6
CMAKE_MINIMUM_REQUIRED(VERSION 3.6)
project(pclBackgroundSegmentation)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( PCL REQUIRED )
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(BackgroundSegmentation BackgroundSegmentation.cpp)

TARGET_LINK_LIBRARIES(BackgroundSegmentation ${PCL_LIBRARIES} -lboost_system)