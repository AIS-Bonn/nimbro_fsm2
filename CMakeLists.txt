
cmake_minimum_required(VERSION 3.2)
project(nimbro_fsm2)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosfmt
	catch_ros
	message_generation
	rqt_gui
	qt_gui_cpp
)

add_message_files(FILES
	StateInfo.msg
	Info.msg
	StateStatus.msg
	Status.msg
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS rosfmt
)

set(CMAKE_CXX_STANDARD 17)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(nimbro_fsm2
	src/detail/format.cpp
)
target_link_libraries(nimbro_fsm2
	${catkin_LIBRARIES}
)
add_dependencies(nimbro_fsm2
	${PROJECT_NAME}_generate_messages_cpp
)

add_subdirectory(example)
add_subdirectory(gui)

if(CATKIN_ENABLE_TESTING)
	catch_add_test(unit_tests
		test/detail/type_name.cpp
		test/detail/variant_membership.cpp
	)
	target_link_libraries(unit_tests
		${catkin_LIBRARIES}
		nimbro_fsm2
	)
	add_dependencies(unit_tests
		${PROJECT_NAME}_generate_messages_cpp
	)

	# Integration tests
	find_package(rostest REQUIRED)
	catch_add_rostest_node(test_simple_fsm
		test/simple_fsm.cpp
	)
	target_link_libraries(test_simple_fsm
		${catkin_LIBRARIES}
		nimbro_fsm2
	)
	add_rostest(test/simple_fsm.test DEPENDENCIES test_simple_fsm)
endif()
