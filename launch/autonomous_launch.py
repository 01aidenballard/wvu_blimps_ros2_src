from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='manual_control',
			name='Manual_Joy',
			executable='joy_to_esc'
		),
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver'
		),
		Node(
			package='joy',
			executable='game_controller_node',
			name='joy_con'
        ),
		Node(
			package='sensors',
			executable='balloon_detect',
			name='balloon_detection',
        ),
		Node(
			package='controls',
			executable='balloon_detect_control',
			name='balloon_detect_PI',
        ),
		Node(
			package='controls',
			executable='mode_switch',
			name='mode_switcher',
        )
	])
def main(args=None):
	generate_launch_description()

if __name__ == "__main__":
	main()
