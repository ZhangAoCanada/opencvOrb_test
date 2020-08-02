cmake_minimum_required(VERSION 3.0)
project(morb)

# Library OpenCV
find_package(OpenCV REQUIRED)

# Library Eigen3
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Library Pangolin for 3D visualization
find_package(Pangolin REQUIRED)

set(INCLUDE_DIRECTORY ${PROJECT_SOURCE_DIR}/include)
set(SOURCE_DIRECTORY ${PROJECT_SOURCE_DIR}/include)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRECTORY})
include_directories(${SOURCE_DIRECTORY})
include_directories(${Pangolin_INCLUDE_DIRS})

file(GLOB mono_files
	"include/*.h"
	"src/*.cpp"
)

# add_executable(morb ${mono_files})
add_executable(morb src/draw_test.cpp ${mono_files})
target_link_libraries(morb ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Eigen3::Eigen)
