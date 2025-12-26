cmake_minimum_required(VERSION 3.5)
project(offboard_control_cpp)

# ===============================
# 🔧 基本配置
# ===============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===============================
# 📦 查找依赖包
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# ===============================
# 📁 头文件目录
# ===============================
include_directories(
  include
  ${px4_msgs_INCLUDE_DIRS}
)

# ===============================
# 🧱 构建库 (liboffboard_control_lib.so)
# ===============================
add_library(offboard_control_lib
  src/offboard_control_lib/offboard_control.cpp
  src/offboard_control_lib/vehicle.cpp
)

# 链接依赖库
target_link_libraries(offboard_control_lib
  PUBLIC
  rclcpp::rclcpp
  ${px4_msgs_TARGETS}   # 或 px4_msgs::px4_msgs
)

# ===============================
# 🚀 构建主程序 (main)
# ===============================
add_executable(offboard_control_main src/main.cpp)

# main 链接自定义库
target_link_libraries(offboard_control_main PRIVATE offboard_control_lib)

# ===============================
# 📦 安装配置
# ===============================
# 安装可执行文件（ros2 run 找这个）
install(TARGETS
  offboard_control_main
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库文件（供其他包复用）
install(TARGETS
  offboard_control_lib
  DESTINATION lib
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# ===============================
# ✅ ament 收尾
# ===============================
ament_package()
