cmake_minimum_required(VERSION 3.8)
project(hetero_fleet_gz)

find_package(ament_cmake REQUIRED)

# `models` ships the converted scan (geomap_colour_noroof) that the world
# includes through model:// - Gazebo resolves that against GZ_SIM_RESOURCE_PATH,
# which gz_sim.launch.py points at this share directory.
install(DIRECTORY worlds launch models
        DESTINATION share/${PROJECT_NAME})

ament_package()
