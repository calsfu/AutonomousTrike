import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # brake controller
        launch_ros.actions.Node(
            package='trike', 
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ),
        # steering controller
        launch_ros.actions.Node(
            package='trike',  
            executable='steering_controller',
            name='steering_controller',
            output='screen'
        ),
        #audio
        launch_ros.actions.Node(
            package='trike', 
            executable='audio_player.py',
            name='audio_player',
            output='screen'
        ),
        # emergency brake
        launch_ros.actions.Node(
            package='trike', 
            executable='emergency_stop.py',
            name='emergency_stop',
            output='screen'
        ),
        # data manager
        launch_ros.actions.Node(
            package='trike', 
            executable='data_manager',
            name='data_manager',
            output='screen'
        ),
        # mode manager
        launch_ros.actions.Node(
            package='trike', 
            executable='mode_manager.py',
            name='mode_manager',
            output='screen'
        ),
        # speech to text
        launch_ros.actions.Node(
            package='trike', 
            executable='speech_to_text.py',
            name='speech_to_text',
            output='screen'
        ),
    ])
