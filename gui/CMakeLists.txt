
find_package(Qt5Widgets REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

qt5_wrap_ui(UI_HDRS
	statemachine_gui.ui
)

qt5_wrap_cpp(MOC_SRCS
	statemachine_gui.h
	timeline.h
	node_graph.h
)

add_library(nimbro_fsm2_gui
	${UI_HDRS}
	${MOC_SRCS}
	statemachine_gui.cpp
	timeline.cpp
	node_graph.cpp
	b_splines.cpp
)
target_link_libraries(nimbro_fsm2_gui
	${catkin_LIBRARIES}
	Qt5::Widgets
)
add_dependencies(nimbro_fsm2_gui
	nimbro_fsm2_generate_messages_cpp
)

install(TARGETS nimbro_fsm2_gui
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES ${PROJECT_SOURCE_DIR}/rqt_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

