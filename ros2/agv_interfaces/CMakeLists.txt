cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReplenishmentOrder.msg"
  "msg/AGVStatus.msg"
  "msg/ShopInventory.msg"
  DEPENDENCIES std_msgs
)

ament_package()
