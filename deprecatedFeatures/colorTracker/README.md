# TO RUN
# 1. go into vmbuild folder in command prompt
# 2. type "cmake .."
# 3. type "make"
# 4. type "./colortracker"

#### will require opencv in local computer to run. will have to modify path to your opencv first before it can run on your computer.

- specifically change these lines

```
get_filename_component(Cyberphysical_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)

set(OpenCV_DIR  ${Cyberphysical_DIR}/opencv-4.0.1) 
```
