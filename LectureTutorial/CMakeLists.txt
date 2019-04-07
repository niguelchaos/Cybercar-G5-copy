cmake_minimum_required(VERSION 3.2)

project(helloworld)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-error=maybe-uninitialized")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# THIS STUFF ONLY FOR OPENCV
# get_filename_component(Cyberphysical_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)
# set(OpenCV_DIR  ${Cyberphysical_DIR}/opencv-4.0.1)

# message("Cyberphysical_DIR = ${Cyberphysical_DIR}")
# message("OpenCV_DIR = ${OpenCV_DIR}")

# find_package(OpenCV REQUIRED)

# Extract cluon-msc.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/cluon-complete-v0.0.121.hpp ${CMAKE_BINARY_DIR}/cluon-complete.hpp
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
    COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc ${CMAKE_BINARY_DIR}/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cluon-complete-v0.0.121.hpp)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/messages.hpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp --out=${CMAKE_BINARY_DIR}/messages.hpp ${CMAKE_CURRENT_SOURCE_DIR}/messages.odvd
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/messages.odvd ${CMAKE_BINARY_DIR}/cluon-msc)
# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})

# OPENCV STUFF
# INCLUDE_DIRECTORIES(../../${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(${OpenCV_INCLUDE_DIRS})

# messages.hpp also needs to be included in the exec as well
# Create actual helloworld executable
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/helloworld.cpp
${CMAKE_CURRENT_SOURCE_DIR}/PrimeChecker.cpp ${CMAKE_BINARY_DIR}/messages.hpp)
target_link_libraries(${PROJECT_NAME} Threads::Threads)

# Create testing executable
enable_testing()
add_executable(${PROJECT_NAME}-Runner TestPrimeChecker.cpp ${CMAKE_CURRENT_SOURCE_DIR}/PrimeChecker.cpp
${CMAKE_BINARY_DIR}/messages.hpp)
target_link_libraries(${PROJECT_NAME}-Runner Threads::Threads)
add_test(NAME ${PROJECT_NAME}-Runner COMMAND ${PROJECT_NAME}-Runner)
# target_link_libraries(${PROJECT_NAME} ${LIBRARIES} rt ${OpenCV_LIBS})
