cmake_minimum_required(VERSION 3.2)

project(colortracker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-error=maybe-uninitialized")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# the path to opencv will have to be changed according to where OpenCVConfig.cmake is,
# which is in opencv-4.0.1/build folder.
# setting where OpenCV_DIR is, the root of the opencv folder, is enough.
get_filename_component(Cyberphysical_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)
set(OpenCV_DIR  ${Cyberphysical_DIR}/opencv-4.0.1)

message("Cyberphysical_DIR = ${Cyberphysical_DIR}")
message("OpenCV_DIR = ${OpenCV_DIR}")

find_package(OpenCV REQUIRED)

# Extract cluon-msc.
# add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
#     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
#     COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/cluon-complete-v0.0.121.hpp ${CMAKE_BINARY_DIR}/cluon-complete.hpp
#     COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
#     COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc ${CMAKE_BINARY_DIR}/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC
#     DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cluon-complete-v0.0.121.hpp)
#
# add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/messages.hpp
#     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
#     COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp --out=${CMAKE_BINARY_DIR}/messages.hpp ${CMAKE_CURRENT_SOURCE_DIR}/messages.odvd
#     DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/messages.odvd ${CMAKE_BINARY_DIR}/cluon-msc)
# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(../../${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/colortrackerexample.cpp)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES} rt ${OpenCV_LIBS})

# ${CMAKE_CURRENT_SOURCE_DIR}/PrimeChecker.cpp ${CMAKE_BINARY_DIR}/messages.hpp)

# enable_testing()
# add_executable(${PROJECT_NAME}-Runner TestPrimeChecker.cpp ${CMAKE_CURRENT_SOURCE_DIR}/PrimeChecker.cpp ${CMAKE_BINARY_DIR}/messages.hpp)
# target_link_libraries(${PROJECT_NAME}-Runner Threads::Threads)
# add_test(NAME ${PROJECT_NAME}-Runner COMMAND ${PROJECT_NAME}-Runner)
