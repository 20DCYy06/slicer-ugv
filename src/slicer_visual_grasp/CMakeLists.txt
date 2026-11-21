cmake_minimum_required(VERSION 3.0.2)
project(slicer_visual_grasp)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(PROGRAMS run.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.json" PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install the same serial implementations and defaults used by the manual tools.
set(workspace_root "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(support "${CATKIN_PACKAGE_SHARE_DESTINATION}/support")
install(FILES
  "${workspace_root}/Hardware/Host/chassis.py"
  "${workspace_root}/Hardware/Host/protocol.py"
  "${workspace_root}/Hardware/Host/transport.py"
  DESTINATION "${support}/Hardware/Host"
)
install(FILES "${workspace_root}/Hardware/config.yaml"
  DESTINATION "${support}/Hardware"
)
install(FILES "${workspace_root}/tools/manual/serial_group.sh"
  DESTINATION "${support}/tools/manual"
)
install(FILES
  "${workspace_root}/tools/manual/b15_servo/keyboard.py"
  "${workspace_root}/tools/manual/b15_servo/config.json"
  DESTINATION "${support}/tools/manual/b15_servo"
)
install(FILES
  "${workspace_root}/tools/manual/airgrip/__init__.py"
  "${workspace_root}/tools/manual/airgrip/keyboard.py"
  "${workspace_root}/tools/manual/airgrip/protocol.py"
  "${workspace_root}/tools/manual/airgrip/config.json"
  DESTINATION "${support}/tools/manual/airgrip"
)
