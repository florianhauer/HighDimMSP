
add_executable (main main.cpp)
TARGET_LINK_LIBRARIES(main kshortestpaths)

add_executable (main2d main2D.cpp)
TARGET_LINK_LIBRARIES(main2d kshortestpaths)

install(DIRECTORY . DESTINATION include/msp
	FILES_MATCHING PATTERN "*.h")
install(DIRECTORY . DESTINATION include/msp
	FILES_MATCHING PATTERN "*.hpp")