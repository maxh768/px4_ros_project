from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, LogInfo, OpaqueFunction)
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os
from datetime import datetime
from launch.conditions import IfCondition
import glob, shutil, subprocess, re

# Topics recorded on every flight.
RECORD_TOPICS = [
    '/fmu/out/vehicle_local_position_v1',
    '/fmu/out/vehicle_attitude',
    '/fmu/out/vehicle_status_v4',
    '/fmu/out/vehicle_control_mode',
    '/fmu/out/vehicle_command_ack_v1',
    '/fmu/out/failsafe_flags',
    '/fmu/out/timesync_status',
    '/fmu/in/offboard_control_mode',
    '/fmu/in/trajectory_setpoint',
    '/fmu/in/vehicle_command',
]

def _archive_ulog(context, *args, **kwargs):
    px4_dir = context.perform_substitution(LaunchConfiguration('px4_dir'))
    dest    = context.perform_substitution(LaunchConfiguration('log_dir'))
    name    = context.perform_substitution(LaunchConfiguration('log_name'))

    logs = glob.glob(os.path.join(px4_dir, 'build/px4_sitl_default/rootfs/log/*/*.ulg'))
    if not logs:
        print('[archive_ulog] no .ulg found')
        return

    src  = max(logs, key=os.path.getmtime)
    stem = os.path.splitext(
        f'{os.path.basename(os.path.dirname(src))}_{os.path.basename(src)}')[0]
    safe = re.sub(r'[^A-Za-z0-9_.-]', '_', name)
    target = os.path.join(dest, f'{stem}_{safe}.ulg' if safe else f'{stem}.ulg')

    os.makedirs(dest, exist_ok=True)
    if not os.path.exists(target):
        shutil.copy2(src, target)

    link = os.path.join(dest, 'latest.ulg')
    if os.path.islink(link) or os.path.exists(link):
        os.remove(link)
    os.symlink(target, link)
    print(f'[archive_ulog] saved {target}')



def _kill_gz(context, *args, **kwargs):
    subprocess.run(['pkill', '-f', 'gz sim'])


def generate_launch_description():
    model    = LaunchConfiguration('model')
    headless = LaunchConfiguration('headless')
    px4_dir  = LaunchConfiguration('px4_dir')
    log_dir  = LaunchConfiguration('log_dir')
    log_name = LaunchConfiguration('log_name')



    # for recording with ros bag
    bag_dir = LaunchConfiguration('bag_dir')
    # Evaluated once, when the launch file is loaded.
    stamp = datetime.now().strftime('%Y-%m-%d_%H_%M_%S')

    bag_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', [bag_dir, '/', stamp, '_', log_name]] + RECORD_TOPICS,
        condition=IfCondition(LaunchConfiguration('record')),
        output='screen')


    # Build the actions first, so event handlers can reference them.
    agent_proc = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen')

    px4_proc = ExecuteProcess(
        cmd=['make', '-C', px4_dir, 'px4_sitl', model],
        additional_env={'HEADLESS': headless,
                        'PX4_PARAM_CBRK_SUPPLY_CHK': '894281',
                        'PX4_PARAM_NAV_DLL_ACT': '0',
                        'PX4_PARAM_SDLOG_PROFILE': '25'},
        output='screen')


    return LaunchDescription([
        DeclareLaunchArgument('log_name', default_value='flight'),
        DeclareLaunchArgument('model',    default_value='gz_x500'),
        DeclareLaunchArgument('headless', default_value=''),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=EnvironmentVariable(
                'PX4_DIR', default_value=os.path.expanduser('~/PX4-Autopilot')),
            description='Path to PX4-Autopilot directory'),
        
        DeclareLaunchArgument( # logs for px4 side
            'log_dir',
            default_value=os.path.join(os.getcwd(), 'flight_logs'),
            description='Where to archive .ulg files after each run'),

        DeclareLaunchArgument( # logs for ros side
            'bag_dir',
            default_value=os.path.join(os.getcwd(), 'rosbags'),
            description='Parent directory for recorded rosbags'),
        DeclareLaunchArgument(
            'record', default_value='true',
            description='Record a rosbag alongside the flight'),


        # init agent
        agent_proc,

        # start px4 + gazebo
        px4_proc,

        # start rosbag recording
        bag_proc,

        #  archive the ulog once PX4 has exited and closed the file
        RegisterEventHandler(OnProcessExit(
            target_action=px4_proc,
            on_exit=[OpaqueFunction(function=_archive_ulog)],
        )),

        RegisterEventHandler(OnShutdown(on_shutdown=[
            LogInfo(msg='shutting down orphaned gz sim processes'),
            OpaqueFunction(function=_kill_gz),
        ])),


    ])
