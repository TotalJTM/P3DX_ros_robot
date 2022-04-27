
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    globalparams = {
        "port":"/dev/ttyACM0",
        "baud":115200,
        "pub_freq":100,
        "freqs/motor_mixer":20,
        "buttons/timer_freq":20,
        "freqs/update_motor":10,
        "freqs/update_encoder":25,
        "freqs/update_button":10
    }
    return LaunchDescription([
		Node(
            package='p3dx_robot',
            executable='robotserial',
            name='robot_communication_node',
            parameters=[{
				"port":"/dev/ttyACM0",
        		"baud":115200,
        		"pub_freq":100,
				}]
        ),
		Node(
            package='p3dx_robot',
            executable='control',
            name='robot_control_node',
            parameters=[{
				"freqs/motor_mixer":20,
        		"buttons/timer_freq":20,
        		"freqs/update_motor":10,
        		"freqs/update_encoder":25,
        		"freqs/update_button":10,
			}]
        ),
    ])