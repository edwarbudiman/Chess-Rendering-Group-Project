cmake_minimum_required(VERSION 3.10)
project(chessRendering)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Check if we're on macOS (which uses Clang)
if(APPLE)
    # Add a definition that we can use to conditionally include headers
    add_definitions(-DMACOS_BUILD)
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/headerFiles
    ${PROJECT_SOURCE_DIR}/src
    )

#add extra .cpp and .hpp files here
add_library(supplemental SHARED
    src/loadModel.cpp
    src/meshChecker.cpp
    src/rasterizer.cpp
    src/scene.cpp
    src/shader.cpp
)

add_executable(chessRenderer main.cpp)
target_link_libraries(supplemental ${OpenCV_LIBRARIES})
target_link_libraries(chessRenderer ${OpenCV_LIBRARIES} supplemental)

add_executable(checkMesh src/meshChecker.cpp)
target_link_libraries(checkMesh ${OpenCV_LIBRARIES} supplemental)

#add more executables as required for testing:
#example:
#add_executable(test1 test1.cpp ${OpenCV_LIBRARIES} supplemental)
#target_link_libraries(test1 ${OpenCV_LIBRARIES} supplemental) 