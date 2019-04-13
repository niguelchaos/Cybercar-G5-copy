cmake_minimum_required(VERSION 3.2)

project(local-safe-distance)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-error=maybe-uninitialized -Wno-error=unused-variable")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# the path to opencv will have to be changed according to where OpenCVConfig.cmake is,
# which is in opencv-4.0.1/build folder.
# setting where OpenCV_DIR is, the root of the opencv folder, is enough.
get_filename_component(Cyberphysical_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../" ABSOLUTE)
set(OpenCV_DIR  ${Cyberphysical_DIR}/opencv-4.0.1)

message("Cyberphysical_DIR = ${Cyberphysical_DIR}")
message("OpenCV_DIR = ${OpenCV_DIR}")

find_package(OpenCV REQUIRED)


include_directories(SYSTEM ${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(../../${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/local-safe-distance.cpp)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES} rt ${OpenCV_LIBS})
