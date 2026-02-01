from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    sms_pvid = ExecuteProcess(
        cmd=[
            "smsrun",
            "pvid",
            "video_path=sms::ClickTrackVideoDemo.mp4",
            "fps=15",
        ],
        output="screen",
    )

    sms_click_ctl = ExecuteProcess(
        cmd=["smsrun", "pclicktrackctl"],
        output="screen",
    )

    sms_detect = ExecuteProcess(
        cmd=[
            "smsrun",
            "pyolo11rk",
            "--job-name=click_detect",
            "model_path=sms::visdrone2019_det-yolo11n_i640_c10-20250731.rknn",
            "confidence=0.4",
            "imgsz=[640,640]",
            "dataset_name=visdrone2019_det",
            "realtime_det=1",
        ],
        output="screen",
    )

    sms_track = ExecuteProcess(
        cmd=[
            "smsrun",
            "pocvsot",
            "--job-name=click_track",
            "show_selection_win=0",
            "realtime_det=1",
        ],
        output="screen",
    )

    bridge_node = Node(
        package="px4_hexctl",
        executable="sms_click_target_bridge",
        name="sms_click_target_bridge",
        output="screen",
        parameters=[{
            "target_x": 0.0,
            "target_y": 0.0,
            "target_z": 0.0,
            "target_valid": False,
            "frame_id": "base_link",
            "publish_rate_hz": 20.0,
            "target_topic": "/qr_tracker/expected_position",
            "publish_cmd_vel": True,
            "cmd_vel_topic": "/qr_tracker/cmd_vel_body",
            "kpx_track": 0.2,
            "kpy_track": 0.2,
            "kpz_track": 0.2,
            "tracking_delta_x": 2.0,
            "tracking_delta_y": 0.0,
            "tracking_delta_z": 0.0,
            "track_z": False,
            "max_vxy": 1.0,
            "max_vz": 0.8,
        }],
    )

    offboard_node = Node(
        package="px4_hexctl",
        executable="offboard_tracker_manager",
        name="offboard_tracker_manager",
        output="screen",
        parameters=[{
            "cmd_vel_topic": "/qr_tracker/cmd_vel_body",
            "takeoff_alt": 1.5,
            "command_timeout": 0.5,
            "enable_adaptive_liftoff": False,
        }],
    )

    return LaunchDescription([
        sms_pvid,
        TimerAction(period=0.5, actions=[sms_click_ctl]),
        TimerAction(period=1.0, actions=[sms_detect]),
        TimerAction(period=1.5, actions=[sms_track]),
        TimerAction(period=2.0, actions=[offboard_node]),
        TimerAction(period=2.5, actions=[bridge_node]),
    ])
