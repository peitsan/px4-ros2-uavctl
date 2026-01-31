cmake_minimum_required(VERSION 3.5)
project(px4_hexctl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	src/px4_hexctl/include
	${px4_msgs_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_library(offboard_control_lib
	src/px4_hexctl/src/offboard_control_lib/offboard_control.cpp
	src/px4_hexctl/src/offboard_control_lib/vehicle.cpp
)

target_link_libraries(offboard_control_lib
	PUBLIC
	rclcpp::rclcpp
	${px4_msgs_TARGETS}
	Eigen3::Eigen
)

add_executable(offboard_takeoff
	src/px4_hexctl/src/offboard_takeoff.cpp
)

add_executable(offboard_circle
	src/px4_hexctl/src/offboard_circle.cpp
)

add_executable(qr_tracker
	src/px4_hexctl/src/qr_tracker.cpp
)

add_executable(qr_tracker_alt
	src/px4_hexctl/src/qr_tracker_alt.cpp
)

target_link_libraries(offboard_takeoff PRIVATE offboard_control_lib)
target_link_libraries(offboard_circle PRIVATE offboard_control_lib)
target_link_libraries(qr_tracker 
	PRIVATE 
	offboard_control_lib 
	${OpenCV_LIBRARIES}
	cv_bridge::cv_bridge
)
target_link_libraries(qr_tracker_alt
	offboard_control_lib
)

install(TARGETS
	offboard_takeoff
	offboard_circle
	qr_tracker
	qr_tracker_alt
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	offboard_control_lib
	DESTINATION lib
)

install(DIRECTORY src/px4_hexctl/include/
	DESTINATION include/
)

install(PROGRAMS
	src/py_script/main.py
	src/py_script/main2.py
	src/py_script/main_srv.py
	src/py_script/offboard_control_lib.py
	src/py_script/test_rect.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
