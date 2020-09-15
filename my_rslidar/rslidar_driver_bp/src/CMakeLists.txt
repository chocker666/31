add_library(rslidar_input_bp input_bp.cc)
target_link_libraries(rslidar_input_bp
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES})

add_library(rslidar_driver_bp rsdriver_bp.cpp)
target_link_libraries(rslidar_driver_bp
  rslidar_input_bp
  ${catkin_LIBRARIES})

# build the nodelet version
add_library(driver_nodelet_bp nodelet_bp.cc rsdriver_bp.cpp)
target_link_libraries(driver_nodelet_bp
  rslidar_input_bp
  ${catkin_LIBRARIES}
)

add_executable(rslidar_node_bp rslidar_node_bp.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_input_bp ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(rslidar_node_bp
	rslidar_driver_bp
  rslidar_input_bp
	${catkin_LIBRARIES}
	${libpcap_LIBRARIES}
)

