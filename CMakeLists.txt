cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")
project(morb)

# Library OpenCV
find_package(OpenCV REQUIRED)
# Library Eigen3
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# Library Pangolin for 3D visualization
find_package(Pangolin REQUIRED)

set(INCLUDE_DIRECTORY ${PROJECT_SOURCE_DIR}/include)
set(SOURCE_DIRECTORY ${PROJECT_SOURCE_DIR}/src)
file(GLOB include_files ${INCLUDE_DIRECTORY}/*.h)
file(GLOB source_files ${SOURCE_DIRECTORY}/*.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRECTORY})

add_executable(morb ${source_files} ${include_files})
target_link_libraries(morb ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Eigen3::Eigen)
