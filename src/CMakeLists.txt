
add_executable (main main.cpp)
TARGET_LINK_LIBRARIES(main)

add_executable (main2d main2D.cpp)
TARGET_LINK_LIBRARIES(main2d)

add_executable (sampling2D sampling2D.cpp)
TARGET_LINK_LIBRARIES(sampling2D)

add_executable (test test.cpp)
TARGET_LINK_LIBRARIES(test)

install(DIRECTORY . DESTINATION include/msp
	FILES_MATCHING PATTERN "*.h")
install(DIRECTORY . DESTINATION include/msp
	FILES_MATCHING PATTERN "*.hpp")