from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    beamng_bridge = Node(
        name='beamng_bridge',
        package='beamng_ros2',
        executable='beamng_bridge',
        output='screen',
        emulate_tty=True,
    )

    scenario_starter = ExecuteProcess(
        cmd = ['ros2',
               'service',
               'call',
               '/beamng_bridge/start_scenario',
               'beamng_msgs/srv/StartScenario',
               '{path_to_scenario_definition: \'/ADC_27/launch/scenario_launch/silesia_scenario.json\'}'
            ],
        output='screen'
    )

    beamng_agent = Node(
        name='beamng_agent',
        package='beamng_agent',
        executable='beamng_agent',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'host': '127.0.0.1'},
            {'port': 25252},
            {'driving_mode': 'keyboard'},
            {'vehicle_id': 'ego'},
            {'vehicle_type': 'sedan'}
        ]
    )

    wait_for_setup = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=scenario_starter,
            on_exit=[beamng_agent]
        )
    )

    return LaunchDescription([
        beamng_bridge,
        scenario_starter,
        wait_for_setup
    ])