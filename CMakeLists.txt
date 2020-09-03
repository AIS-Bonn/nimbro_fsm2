
cmake_minimum_required(VERSION 3.2)

# We default to clang compiler in order to use our own clang plugin.
if(NOT CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
	find_program(CLANG_EXECUTABLE NAMES clang-9 clang-8 clang-7 clang-6 clang)
	find_program(CLANGPP_EXECUTABLE NAMES clang++-9 clang++-8 clang++-7 clang++-6 clang++)
	if(CLANG_EXECUTABLE)
		set(CMAKE_C_COMPILER ${CLANG_EXECUTABLE})
		set(CMAKE_CXX_COMPILER ${CLANGPP_EXECUTABLE})
	else()
		message(WARNING "Install clang to get better diagnostics!")
	endif()
else()
	message(STATUS "CMAKE_CXX_COMPILER already set to ${CMAKE_CXX_COMPILER}")
endif()

project(nimbro_fsm2)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosfmt
	catch_ros
	message_generation
	rqt_gui
	qt_gui_cpp
	actionlib
	actionlib_msgs
)

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
	message(STATUS "Clang version: ${CMAKE_CXX_COMPILER_VERSION}")
	if("${CMAKE_CXX_COMPILER_VERSION}" MATCHES "^([0-9]+)\\.([0-9]+).*$")
		set(CLANG_VERSION_MAJOR ${CMAKE_MATCH_1})
		set(CLANG_VERSION_MINOR ${CMAKE_MATCH_2})
	else()
		message(WARNING "Could not parse Clang version string")
		set(CLANG_VERSION_MAJOR "unknown")
		set(CLANG_VERSION_MINOR "unknown")
	endif()

	find_path(CLANG_INCLUDE_DIRS "clang/Basic/Version.h" PATHS
		"/usr/lib/llvm-${CLANG_VERSION_MAJOR}/include"
		"/usr/lib/llvm-${CLANG_VERSION_MAJOR}.${CLANG_VERSION_MINOR}/include"
	)
endif()

# NOTE: HAVE_PLUGIN is used below in CFG_EXTRAS
if(CLANG_INCLUDE_DIRS)
	set(HAVE_PLUGIN "YES")
	message(STATUS "Found clang headers at ${CLANG_INCLUDE_DIRS}")
else()
	set(HAVE_PLUGIN "NO")
	message(WARNING "Install clang headers (Ubuntu: libclang-dev) for better diagnostics!")
endif()

add_message_files(FILES
	StateInfo.msg
	Info.msg
	StateStatus.msg
	Status.msg
)

add_action_files(FILES
	ChangeState.action
)

generate_messages(DEPENDENCIES
	std_msgs
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS rosfmt
	CFG_EXTRAS nimbro_fsm2.cmake
)

# NOTE: We have to compile the plugin after catkin_package() to make sure it
# ends up in the right place.
if(HAVE_PLUGIN STREQUAL "YES")
	add_subdirectory(clang_plugin)
endif()

set(CMAKE_CXX_STANDARD 17)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(nimbro_fsm2
	src/detail/ros_interface.cpp
	src/detail/format.cpp
	src/detail/watchdog.cpp
)
target_link_libraries(nimbro_fsm2
	${catkin_LIBRARIES}
)
add_dependencies(nimbro_fsm2
	${PROJECT_NAME}_generate_messages_cpp
)

install(DIRECTORY include/
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(TARGETS nimbro_fsm2
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# If we have the compiler plugin, attach the compiler options for loading to
# the nimbro_fsm2 target. NOTE: This is only used for internal targets,
# for the exported version see cmake/nimbro_fsm2.cmake.in.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND ${HAVE_PLUGIN} STREQUAL "YES")
	message(STATUS "Using clang compiler plugin...")
	target_compile_options(nimbro_fsm2
		PUBLIC "-fplugin=$<TARGET_FILE:fsm_checker>"
	)
	target_compile_definitions(nimbro_fsm2
		PUBLIC "NIMBRO_FSM2_CHECKED=1"
	)
	add_dependencies(nimbro_fsm2 fsm_checker)
else()
	message(WARNING "Consider switching to clang to make use of nimbro_fsm2's "
		"clang plugin for better diagnostics")
endif()

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

	install(TARGETS unit_tests test_simple_fsm
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
	)
endif()

