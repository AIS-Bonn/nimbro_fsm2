
cmake_minimum_required(VERSION 3.2)
project(nimbro_fsm2)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosfmt
	catch_ros
	message_generation
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

add_subdirectory(example)

if(CATKIN_ENABLE_TESTING)
	catch_add_test(unit_tests
		test/detail/type_name.cpp
		test/detail/variant_membership.cpp
	)
	target_link_libraries(unit_tests
		${catkin_LIBRARIES}
	)
endif()
