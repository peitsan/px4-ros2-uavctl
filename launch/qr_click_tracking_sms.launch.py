from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():
    start_smscore = LaunchConfiguration("start_smscore")

    smscore = ExecuteProcess(
        cmd=["smscore"],
        output="screen",
        condition=IfCondition(start_smscore),
    )

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

    bridge_exe = os.path.join(
        get_package_prefix("px4_hexctl"),
        "lib",
        "px4_hexctl",
        "sms_click_target_bridge",
    )
    bridge_node = ExecuteProcess(
        cmd=[
            bridge_exe,
            "--config",
            os.path.expanduser("~/spirecv-pro/params/spirecv2/default_params.json"),
            "--job-name",
            "click_bridge",
            "--ip",
            "127.0.0.1",
            "--port",
            "9094",
            "target_x=0.0",
            "target_y=0.0",
            "target_z=0.0",
            "target_valid=false",
            "frame_id=base_link",
            "publish_rate_hz=20.0",
            "target_topic=/qr_tracker/expected_position",
        ],
        output="screen",
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
        DeclareLaunchArgument(
            "start_smscore",
            default_value="true",
            description="Start SpireMS smscore service before SMS nodes",
        ),
        smscore,
        TimerAction(period=0.5, actions=[sms_pvid]),
        TimerAction(period=0.5, actions=[sms_click_ctl]),
        TimerAction(period=1.0, actions=[sms_detect]),
        TimerAction(period=1.5, actions=[sms_track]),
        TimerAction(period=2.0, actions=[offboard_node]),
        TimerAction(period=2.5, actions=[bridge_node]),
    ])
