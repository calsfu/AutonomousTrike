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
        )
    ])
