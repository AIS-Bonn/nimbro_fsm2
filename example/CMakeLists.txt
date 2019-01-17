
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
