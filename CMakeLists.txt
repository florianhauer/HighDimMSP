# CMakeLists
cmake_minimum_required (VERSION 2.8.5)
project (HDMSP)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )
SET( RESULT_DIRECTORY ${BASE_DIR}/results )


SET( CMAKE_BUILD_TYPE Debug)
SET( CMAKE_CXX_FLAGS  "-Ofast -std=c++0x -Wno-unused-result -DRESDIR2=\"${RESULT_DIRECTORY}\" " )


include_directories("./3rdparty/")
ADD_SUBDIRECTORY( 3rdparty )
ADD_SUBDIRECTORY( src )
