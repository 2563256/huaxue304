cmake_minimum_required(VERSION 3.8)
project(hetero_fleet_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz maps
        DESTINATION share/${PROJECT_NAME})

# Note: the offline map generators (make_map.py / make_shelf_blockers.py in the
# warehouse workspace) are gone - this workspace rasterises a scanned mesh
# instead, with tools/make_nav2_map.py. Only runtime nodes are installed here.
install(PROGRAMS scripts/channel_gate.py scripts/scan_frame_relay.py scripts/gz_pose_tf_relay.py scripts/omni_odom_relay.py scripts/cmd_vel_stamper.py scripts/map_server_watchdog.py scripts/nav2_stack_watchdog.py scripts/rviz_compat.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
