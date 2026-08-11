from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, LogInfo)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
import os

    

def generate_launch_description():
    model    = LaunchConfiguration('model')
    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument('model',    default_value='gz_x500'),
        DeclareLaunchArgument('headless', default_value=''),

        DeclareLaunchArgument(
        'px4_dir',
        default_value=EnvironmentVariable(
            'PX4_DIR',default_value=os.path.expanduser('~/PX4-Autopilot')),
        description='Path to PX4-Autopilot directory'),
        

        # 1. agent first — avoids noisy client retry warnings
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                       output='screen'),

        # 2. PX4 + Gazebo, via make so cwd and gz_env.sh are correct
        ExecuteProcess(
            cmd=['make', '-C', LaunchConfiguration('px4_dir'), 'px4_sitl', model],
            additional_env={'HEADLESS': headless},
            output='screen'),

        # 3. cleanup — PX4 backgrounds gz sim outside our process tree
        RegisterEventHandler(OnShutdown(on_shutdown=[
            LogInfo(msg='shutting down orphaned gz sim processes'),
            ExecuteProcess(cmd=['pkill', '-f', 'gz sim']),
        ])),
    ])
