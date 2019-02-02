
add_executable(demo
	idle.cpp
	driving.cpp
	crashed.cpp
	demo.cpp
)
target_link_libraries(demo
	${catkin_LIBRARIES}
	nimbro_fsm2
)
add_dependencies(demo
	${PROJECT_NAME}_generate_messages_cpp
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	message(STATUS "Using clang compiler plugin...")
	target_compile_options(demo PRIVATE
		"-fplugin=$<TARGET_FILE:fsm_checker>"
	)
	add_dependencies(demo fsm_checker)
endif()
