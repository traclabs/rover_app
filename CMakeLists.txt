project(ROVER_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${ros_app_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(rover_app fsw/src/rover_app.c)
target_link_libraries(rover_app m)

target_include_directories(rover_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
)
