cmake_minimum_required(VERSION 3.8)
project(hetero_fleet_gz)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-physics7 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
add_library(hetero_mecanum_contacts SHARED src/mecanum_contacts.cpp)
target_compile_features(hetero_mecanum_contacts PRIVATE cxx_std_17)
target_link_libraries(hetero_mecanum_contacts PRIVATE gz-sim8::gz-sim8 gz-physics7::gz-physics7 gz-plugin2::register)
install(TARGETS hetero_mecanum_contacts LIBRARY DESTINATION lib)
ament_export_libraries(hetero_mecanum_contacts)

# `models` ships the converted scan (geomap_colour_noroof) that the world
# includes through model:// - Gazebo resolves that against GZ_SIM_RESOURCE_PATH,
# which gz_sim.launch.py points at this share directory.
install(DIRECTORY worlds launch models
        DESTINATION share/${PROJECT_NAME})

ament_package()
