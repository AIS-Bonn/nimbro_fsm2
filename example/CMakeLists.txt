
add_executable(demo
	idle.cpp
	driving.cpp
	crashed.cpp
	demo.cpp
)
target_link_libraries(demo
	${catkin_LIBRARIES}
)
