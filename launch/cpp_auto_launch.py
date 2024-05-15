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
				"Klm": 1.0,
				"Krm": 1.125,
			}]
        ),
		#Sensors Package Excecutable
	#	Node(
	#		package='sensors_cpp',
	#		executable='balloon_detect_cpp',
	#		name='balloon_detection',
       # ),
		Node(
			package='sensors_cpp',
			executable='detect_cpp',
			name='cam_node',
        ),
		#Control Package Executables:
		Node(
			package='sensors_cpp',
			executable='pi_controller',
			name='balloon_detect_PI',
			#kpx = 0.3 without forward motors
			parameters = [{

				"iheight": 4.0,
				"kpx": 0.003, #0.005,
				      #0.00000001
				"kix":  0.000002,
				"kpyu": 0.002, # up motor
				"kpyd": 0.0015, # down motor
				"kiy":  0.00001, #0.000000001,
				"kpb":  0.69  #0.69  #NICE!
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
				"buoyancy": ((0.45131) - (5*0.001))*9.81,
				"rho_air": 1.225
			}]
        ),
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver',
			parameters = [{
				"MAC":"68:6C:E6:73:04:62"
			}]
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
				"sea_level_pressure": 1017.0
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
