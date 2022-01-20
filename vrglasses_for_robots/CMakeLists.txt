cmake_minimum_required(VERSION 2.8.12)
project(vrglasses4robots)

#set(CMAKE_BUILD_TYPE Debug)


#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

set (CMAKE_CXX_STANDARD 17)
#find_package(tinyobjloader REQUIRED)
find_package( Eigen3 REQUIRED )
find_package( OpenCV REQUIRED core highgui imgproc )
find_package( Vulkan )

find_package( Boost COMPONENTS filesystem REQUIRED )

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories( ${TINYOBJLOADER_INCLUDE_DIRS} )


include_directories( include )

#hack to add headers in the qtcreator
FILE(GLOB_RECURSE hearderFiles "include/*.h")
add_custom_target(headers SOURCES ${hearderFiles})

add_library(${PROJECT_NAME}_lib "src/VulkanTools.cc" "src/vulkan_renderer.cc")
target_link_libraries(${PROJECT_NAME}_lib   ${Vulkan_LIBRARY} )

add_executable(${PROJECT_NAME}_csv  "src/csv_main.cc"  "src/csv_processor.cc")
target_link_libraries(${PROJECT_NAME}_csv glog gflags  ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Vulkan_LIBRARY} ${PROJECT_NAME}_lib)


