cmake_minimum_required(VERSION 3.8)
project(solar_farm_gz)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/generate_farm
  scripts/capture
  scripts/teleop_joy
  scripts/flight_video
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

# models/ holds hand-authored assets — the inspection drone and its sensors.
# It is deliberately separate from worlds/: worlds/ is generator output and is
# gitignored wholesale, so anything hand-written placed there would never reach
# the client's repository.
install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME})

# gui/ holds the Gazebo GUI layout used for inspection flights: free-orbit 3D
# view and the nadir camera feed, both docked on startup.
install(DIRECTORY gui
        DESTINATION share/${PROJECT_NAME})

# worlds/ holds the generated world plus the solar_farm_assets model package.
# It is installed wholesale so that the model:// URIs inside the SDF resolve
# against a single GZ_SIM_RESOURCE_PATH entry. The directory is empty in a
# fresh clone until generate_farm has been run.
install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")

ament_package()
