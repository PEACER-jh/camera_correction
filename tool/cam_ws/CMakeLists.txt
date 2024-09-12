cmake_minimum_required(VERSION 3.0)
project(camera_correction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # ${CERES_INCLUDE_DIRS}
    ./include)

set(SOURCES
    ./src/drawHistogram.cpp
)

add_executable(app videoCapture.cpp ${SOURCES})
target_link_libraries(app ${OpenCV_LIBS})
# target_link_libraries(app ${OpenCV_LIBS} ${CERES_LIBRARIES})