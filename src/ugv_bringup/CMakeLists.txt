cmake_minimum_required(VERSION 3.0.2)
project(ugv_bringup)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(UGV_BRINGUP_GENERATED_CONFIG_DIR)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN "online_runtime" EXCLUDE
    PATTERN "footprint.yaml" EXCLUDE
    PATTERN "slicer_driver_body_speed.yaml" EXCLUDE)
  install(DIRECTORY "${UGV_BRINGUP_GENERATED_CONFIG_DIR}/"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
else()
  install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
