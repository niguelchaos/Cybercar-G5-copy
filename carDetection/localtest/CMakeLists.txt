cmake_minimum_required(VERSION 3.2)

project(local-car-detection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-error=maybe-uninitialized -Wno-error=unused-variable")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# the path to opencv will have to be changed according to where OpenCVConfig.cmake is,
# which is in opencv-4.0.1/build folder.
# setting where OpenCV_DIR is, the root of the opencv folder, is enough.
get_filename_component(Cyberphysical_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../" ABSOLUTE)
set(OpenCV_DIR  ${Cyberphysical_DIR}/opencv-4.0.1)
set(OpenCV_INCLUDE_DIRS ${OpenCV_DIR}/include)
set(DOPENCV_EXTRA_MODULES_PATH ${OpenCV_DIR}/opencv_contrib-4.0.1/modules)

message("Cyberphysical_DIR = ${Cyberphysical_DIR}")
message("OpenCV_DIR = ${OpenCV_DIR}")
message("OpenCV include DIR = ${OpenCV_INCLUDE_DIRS}")
message("binary dir = SYSTEM ${CMAKE_BINARY_DIR}")
message("source dir = ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(OpenCV REQUIRED)

################################################################################
# Defining the relevant versions of OpenDLV Standard Message Set and libcluon.
set(OPENDLV_STANDARD_MESSAGE_SET opendlv-standard-message-set-v0.9.6.odvd)
set(CLUON_COMPLETE cluon-complete-v0.0.121.hpp)


################################################################################
# Extract cluon-msc from cluon-complete.hpp.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.hpp
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
    COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc ${CMAKE_BINARY_DIR}/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${CLUON_COMPLETE})

################################################################################
# Generate opendlv-standard-message-set.hpp from ${OPENDLV_STANDARD_MESSAGE_SET} file.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${OPENDLV_STANDARD_MESSAGE_SET}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${OPENDLV_STANDARD_MESSAGE_SET} ${CMAKE_BINARY_DIR}/cluon-msc)
# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


################################################################################
# Gather all object code first to avoid double compilation.
set(LIBRARIES Threads::Threads)

include_directories(SYSTEM ${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(../../${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})


# add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/local-car-detection.cpp)
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp)

target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES} rt ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

################################################################################
# Install executable.
install(TARGETS ${PROJECT_NAME} DESTINATION bin COMPONENT ${PROJECT_NAME})
