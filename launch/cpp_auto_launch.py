from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		#Outside dist packages
		Node(
			package='joy',
			executable='game_controller_node',
			name='joy_con',
			#parameters = [{"autorepeat_rate": 10.0}]
        ),
		#Manual Control Package Excecutable
		Node(
			package='manual_control',
			executable='joy_to_esc',
			name='joy_to_esc',
			parameters = [{
				"Klm": 1.2,
				"Krm": 1.0,
			}]
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
				"kpx": 0.005, #0.005,
				      #0.00000001
				"kix": 0.0, #0000001,
				"kpyu": 0.004, # up motor
				"kpyd": 0.0, # down motor
				"kiy": 0.000001, #0.000000001,
				"kpb": 0.69  #0.69  #NICE!
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
			parameters = [{
				"buoyancy": (0.42511726*9.81) - 0.20601,
				"rho_air": 1.225
			}]
        ),
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver'
		),
		Node(
			package='controls',
			name='net_servo',
			executable='net_servo'
		),
		Node(
			package='sensors',
			name='read_altitude',
			executable='read_altitude',
			parameters = [{
				"sea_level_pressure": 1011.3
			}]
		),
		Node(
			package='sensors',
			name='read_imu',
			executable='read_imu'
		),
		# Node(
		# 	package = 'sensors',
		# 	name = 'record_data',
		# 	executable = 'record_data',
		# 	parameters = [{"file_name":"pi_DBlue_4"}]
		# ),
	#	Node(
	#		package='sensors',
	#		name='sender_data',
	#		executable='sender'
	#	),
		# Node(
		# 	package='sensors',
		# 	name='reciever',
		# 	executable='recieve_data',
		# 	parameters = [{
		# 		"file_name": "test_data_1"
		# 	}]
		# ),
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
