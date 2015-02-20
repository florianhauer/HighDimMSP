# CMakeLists
cmake_minimum_required (VERSION 2.8.5)
project (HDMSP)


SET( CMAKE_BUILD_TYPE Debug)
SET( CMAKE_CXX_FLAGS  "-std=c++0x" )

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

include_directories("./3rdparty/kshortestpaths/")
ADD_SUBDIRECTORY( 3rdparty )
ADD_SUBDIRECTORY( src )
