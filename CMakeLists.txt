cmake_minimum_required(VERSION 3.9)
project(gtsam_gpdepth CXX C)
set(CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(GTSAM REQUIRED) 

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)

set(WRAP_PYTHON_VERSION ${GTSAM_PYTHON_VERSION}
        CACHE STRING "The Python version to use for wrapping" FORCE)

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

add_library(${PROJECT_NAME} SHARED cpp/GpDepthFactorBase.h
                                   cpp/GpDepthFactor2D.h cpp/GpDepthFactor3D.h)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} gtsam)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

include(GtsamBuildTypes)
include(GtsamTesting)
gtsamAddTestsGlob("${PROJECT_NAME}" "cpp/tests/*.cpp" "" "gtsam;${PROJECT_NAME}")

add_subdirectory(python)