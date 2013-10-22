set(the_description "The UbiTrack Video4Linux Camera Component")
ut_add_component(video4linux DEPS utcore utdataflow utvision)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )
# ALSO NEEDS OPENCV FOR IMAGE CLASS !!!
FIND_PACKAGE(V4L2)
IF(V4L2_FOUND)
	SET(HAVE_V4L2 1)
	add_definitions(-DHAVE_V4L2)
	ut_component_include_directories(${TINYXML_INCLUDE_DIR} ${LOG4CPP_INCLUDE_DIR} ${BOOSTBINDINGS_INCLUDE_DIR} ${LAPACK_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${V4L2_INCLUDE_DIRS})
	ut_glob_component_sources(HEADERS "src/Video4Linux/*.h" SOURCES "src/Video4Linux/*.cpp")
	ut_create_multi_component(${V4L2_LIBRARIES})
	ut_install_utql_patterns()
ENDIF(V4L2_FOUND)

