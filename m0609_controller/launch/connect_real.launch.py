from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
import os
import launch_ros

def generate_launch_description():
    package_dir = launch_ros.substitutions.FindPackageShare(package='m0609_controller').find('m0609_controller')
    exe_path = os.path.join(package_dir, 'server')

    return LaunchDescription([
        #Python 스크립트 실행
        ExecuteProcess(
            cmd=['ros2', 'run', 'm0609_controller', 'sub_joint'],
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
        # 실행 파일 실행
                ExecuteProcess(
                    cmd=[exe_path],
                    output='screen'
                )
            ]
        ),
    ])
