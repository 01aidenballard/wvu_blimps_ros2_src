from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		#Outside dist packages
		Node(
			package='joy',
			executable='game_controller_node',
			name='joy_con',
			parameters = [{"autorepeat_rate": 10.0}]
        ),
		#Manual Control Package Excecutable
		Node(
			package='manual_control',
			executable='joy_to_esc',
			name='joy_to_esc'
        ),
		#Sensors Package Excecutable
		Node(
			package='sensors_cpp',
			executable='balloon_detect_cpp',
			name='balloon_detection',
        ),
		#Control Package Executables:
		Node(
			package='sensors_cpp',
			executable='pi_controller',
			name='balloon_detect_PI',
			#kpx = 0.3 without forward motors
			parameters = [{
				"kpx": 0.35,
				"kix": 0.0,
				"kpy": 0.0,
				"kiy": 0.0
			}]
        ),
		Node(
			package='sensors_cpp',
			executable='F_to_Esc',
			name='force_to_esc',
        ),
		Node(
			package='sensors_cpp',
			executable='dynamic_model',
			name='inv_kine',
        ),
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver'
		),
		Node(
			package='sensors',
			name='read_altitude',
			executable='read_altitude'
		),
		Node(
			package='sensors',
			name='read_imu',
			executable='read_imu'
		),
#		Node(
#			package = 'sensors',
#			name = 'record_data',
#			executable = 'record_data',
#			parameters = [{"file_name":"test_data"}]
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
