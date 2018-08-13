cmake_minimum_required( VERSION 2.8 )
project( VO )
# Release mode
set( CMAKE_BUILD_TYPE Release)
# Debug mode 
# SET(CMAKE_BUILD_TYPE "Debug")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# opencv 
find_package( OpenCV 4.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )



find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
)

include_directories(${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )


