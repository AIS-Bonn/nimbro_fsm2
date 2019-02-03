
cmake_minimum_required(VERSION 3.2)

# We default to clang compiler in order to use our own clang plugin.
if(NOT CMAKE_CXX_COMPILER)
	find_program(CLANG_EXECUTABLE clang)
	find_program(CLANGPP_EXECUTABLE clang++)
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

# First try to find ClangConfig.cmake
find_package(Clang QUIET)

# On clang 6.0, this fails (at least on Ubuntu)
if(NOT CLANG_INCLUDE_DIRS)
	find_path(CLANG_INCLUDE_DIRS "clang/Basic/Version.h" PATHS "/usr/lib/llvm-6.0/include")
endif()

if(CLANG_INCLUDE_DIRS)
	set(HAVE_PLUGIN "YES")
	add_subdirectory(clang_plugin)
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

set(CMAKE_CXX_STANDARD 17)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(nimbro_fsm2
	src/detail/action_interface.cpp
	src/detail/format.cpp
	src/detail/watchdog.cpp
)
target_link_libraries(nimbro_fsm2
	${catkin_LIBRARIES}
)
add_dependencies(nimbro_fsm2
	${PROJECT_NAME}_generate_messages_cpp
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
endif()

