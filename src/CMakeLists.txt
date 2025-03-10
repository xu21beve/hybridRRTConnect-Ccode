find_package(ompl REQUIRED)
# find_package(ament_cmake_core REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# find_package(rosidl_typesupport_c REQUIRED)

add_library(cHyRRT HyRRT.cpp)
add_library(HybridStateSpace HybridStateSpace.cpp)
add_library(HybridTimeStateSpace HybridTimeStateSpace.cpp)
target_link_libraries(cHyRRT HybridStateSpace HybridTimeStateSpace ${OMPL_LIBRARIES})

set(CMAKE_TOOLCHAIN_FILE ${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)

find_package(Boost REQUIRED
             COMPONENTS serialization filesystem system program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})

include_directories(${HyRRT_SOURCE_DIR}) # Include the directory where HyRRT.h is located

# Include necessary directories
include_directories(${OMPL_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")