add_library(rslidar_data_bp rawdata_bp.cc)
target_link_libraries(rslidar_data_bp
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(rslidar_point_bp convert_bp.cc)
target_link_libraries(rslidar_point_bp
    rslidar_data_bp
    ${catkin_LIBRARIES})


add_library(cloud_nodelet_bp cloud_nodelet_bp.cc)
target_link_libraries(cloud_nodelet_bp rslidar_point_bp
                      ${catkin_LIBRARIES})

add_executable(cloud_node_bp cloud_node_bp.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_data_bp ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(cloud_node_bp
    rslidar_point_bp
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})
