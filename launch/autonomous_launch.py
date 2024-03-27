from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		#Outside dist packages
		Node(
			package='joy',
			executable='game_controller_node',
			name='joy_con',
			parameters = [{"autorepeat_rate": 5.0}]
        ),
		#Manual Control Package Excecutable
		Node(
			package='manual_control',
			executable='joy_to_esc',
			name='joy_to_esc'
        ),
		#Sensors Package Excecutable
		Node(
			package='sensors',
			executable='balloon_detect',
			name='balloon_detection',
        ),
		#Control Package Executables:
		Node(
			package='controls',
			executable='balloon_detect_control',
			name='balloon_detect_PI',
        ),
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver'
		),
#		Node(
#			package = 'sensors',
#			name = 'record_data',
#			executable = 'record_data'
#		),
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
