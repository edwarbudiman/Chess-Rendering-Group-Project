cmake_minimum_required(VERSION 3.10)
project(chessRendering)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/headerFiles
    ${PROJECT_SOURCE_DIR}/src
    "${PROJECT_SOURCE_DIR}/labCode/Danny'sLabCode/HW3/include"  # <-- add this
)

#add extra .cpp and .hpp files here
add_library(supplemental SHARED
    src/loadModel.cpp
    src/meshChecker.cpp
    src/renderer.cpp
)

add_executable(chessRenderer main.cpp)
target_link_libraries(chessRenderer ${OpenCV_LIBRARIES} supplemental)

# Test program for the OBJ loader and half-edge conversion
add_executable(test_loader src/test_loader.cpp)
target_link_libraries(test_loader ${OpenCV_LIBRARIES} supplemental)

#add more executables as required for testing:
#example:
#add_executable(test1 test1.cpp ${OpenCV_LIBRARIES} supplemental)
#target_link_libraries(test1 ${OpenCV_LIBRARIES} supplemental)