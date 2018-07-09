cmake_minimum_required(VERSION 2.8.3)
project(dataset_building)

if(DEBUG)
    set(CMAKE_BUILD_TYPE "Debug")
    set(CMAKE_CXX_FLAGS "-std=c++11 -O0 ${CMAKE_CXX_FLAGS}")
else()
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "-std=c++11 -O3 ${CMAKE_CXX_FLAGS}")
endif()

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED bfl)

find_package(image_geometry)

find_package(Boost COMPONENTS program_options system)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/local/include
)

link_directories(/usr/local/lib)

include_directories(${BFL_INCLUDE_DIRS})
message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(
  GLOB_RECURSE
  source_files
  src/*/*
)

add_executable(dataset_builder
  ${source_files}
  src/dataset_builder.cpp
)

target_link_libraries(dataset_builder
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${BFL_LIBRARIES}
  ${image_geometry_LIBRARIES}
  ${Boost_LIBRARIES}
  tbb
  image_processing
  iagmm
)






