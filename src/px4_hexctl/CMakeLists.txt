cmake_minimum_required(VERSION 3.5)
project(px4_hexctl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_library(SMS_CORE_LIBRARY sms_core PATHS /usr/local/lib /usr/lib)
find_path(SPIRECV_INCLUDE_DIR sv_world.h PATHS /usr/local/include /usr/include /opt/SpireCV/include /home/ubuntu/SpireCV/include /root/SpireCV/include /home/ubuntu/spirecv-pro/include /root/spirecv-pro/include $ENV{HOME}/spirecv-pro/include)
find_library(SPIRECV_LIBRARY spirecv PATHS /usr/local/lib /usr/lib /opt/SpireCV/lib /home/ubuntu/SpireCV/lib /root/SpireCV/lib /home/ubuntu/spirecv-pro/lib /root/spirecv-pro/lib $ENV{HOME}/spirecv-pro/lib)

set(SPIRECV_FOUND FALSE)
if(SPIRECV_INCLUDE_DIR AND SPIRECV_LIBRARY)
	set(SPIRECV_FOUND TRUE)
	message(STATUS "Found SpireCV: ${SPIRECV_INCLUDE_DIR}, ${SPIRECV_LIBRARY}")
else()
	message(WARNING "SpireCV not found. qr_tracker_spirecv will be disabled.")
endif()
find_package(image_view_msgs QUIET)

include_directories(
	include
	${px4_msgs_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_library(offboard_control_lib
	src/offboard_control_lib/offboard_control.cpp
	src/offboard_control_lib/vehicle.cpp
)

target_link_libraries(offboard_control_lib
	PUBLIC
	rclcpp::rclcpp
	${px4_msgs_TARGETS}
	Eigen3::Eigen
)

add_executable(offboard_arm
	src/offboard_arm.cpp
)

target_link_libraries(offboard_arm offboard_control_lib)

add_executable(offboard_takeoff
	src/offboard_takeoff.cpp
)


add_executable(offboard_circle
	src/offboard_circle.cpp
)

add_executable(offboard_tracker_manager
	src/offboard_tracker_manager.cpp
)

add_executable(sms_click_target_bridge
	src/sms_click_target_bridge.cpp
)

add_executable(qr_tracker
	src/qr_tracker.cpp
)

# add_executable(qr_qrcode_opencv
# 	src/qr_qrcode_opencv.cpp
# )

if(SPIRECV_FOUND)
	add_executable(qr_tracker_spirecv
		src/qr_tracker_spirecv.cpp
	)
endif()

add_executable(qr_tracker_alt
	src/qr_tracker_alt.cpp
)

add_executable(takeoff_fixedpoint
	src/takeoff_fixedpoint.cpp
)

# add_executable(offboard_single_position_xrdds
# 	src/offboard_single_position_xrdds.cpp
# )

target_link_libraries(offboard_takeoff PRIVATE offboard_control_lib)

target_link_libraries(offboard_circle PRIVATE offboard_control_lib)
# target_link_libraries(offboard_single_position_xrdds offboard_control_lib)
target_link_libraries(qr_tracker 
	PRIVATE 
	offboard_control_lib 
	${OpenCV_LIBRARIES}
	cv_bridge::cv_bridge
)

# target_link_libraries(qr_qrcode_opencv
# 	${OpenCV_LIBRARIES}
# 	cv_bridge::cv_bridge
# )

# ament_target_dependencies(qr_qrcode_opencv
# 	rclcpp
# 	sensor_msgs
# 	geometry_msgs
# )

ament_target_dependencies(offboard_arm
	rclcpp
	sensor_msgs
	geometry_msgs
)


if(SPIRECV_FOUND)
	target_link_libraries(qr_tracker_spirecv
		PRIVATE
		offboard_control_lib
		${OpenCV_LIBRARIES}
		${SPIRECV_LIBRARY}
		cv_bridge::cv_bridge
	)

	target_include_directories(qr_tracker_spirecv PRIVATE ${SPIRECV_INCLUDE_DIR})

	ament_target_dependencies(qr_tracker_spirecv geometry_msgs)
endif()
ament_target_dependencies(offboard_tracker_manager geometry_msgs)
target_link_libraries(offboard_tracker_manager offboard_control_lib)
if(SMS_CORE_LIBRARY)
	target_include_directories(sms_click_target_bridge PRIVATE /usr/local/include)
	target_link_libraries(sms_click_target_bridge ${SMS_CORE_LIBRARY})
else()
	message(WARNING "SpireMS sms_core library not found. sms_click_target_bridge may fail to link.")
endif()
if(image_view_msgs_FOUND)
  ament_target_dependencies(qr_tracker image_view_msgs)
  target_compile_definitions(qr_tracker PRIVATE HAS_IMAGE_VIEW_MSGS=1)
endif()
target_link_libraries(qr_tracker_alt
	offboard_control_lib
)

ament_target_dependencies(takeoff_fixedpoint
	rclcpp
	px4_msgs
)


# ament_target_dependencies(offboard_single_position_xrdds
# 	rclcpp
# 	px4_msgs
# )

set(PX4_HEXCTL_TARGETS
	offboard_arm
	offboard_takeoff
	offboard_circle
	qr_tracker
	# qr_qrcode_opencv
	qr_tracker_alt
	offboard_tracker_manager
	sms_click_target_bridge
	takeoff_fixedpoint
	# offboard_single_position_xrdds
)
if(SPIRECV_FOUND)
	list(APPEND PX4_HEXCTL_TARGETS qr_tracker_spirecv)
endif()

install(TARGETS
	${PX4_HEXCTL_TARGETS}
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	offboard_control_lib
	DESTINATION lib
)

install(DIRECTORY include/
	DESTINATION include/
)

install(PROGRAMS
	px4_hexctl/vins_offboard_hover.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME vins_offboard_hover
)

install(PROGRAMS
	py_script/main.py
	py_script/main2.py
	py_script/main_srv.py
	py_script/offboard_control_lib.py
	py_script/test_rect.py
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
