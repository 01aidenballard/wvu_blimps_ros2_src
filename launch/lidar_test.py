from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node (
			package='sensors',
			executable='read_lidar',
			name='lidar',
		),

		Node (
			package='joy',
			executable='game_controller_node',
			name='xbox'
		),

		Node (
			package='controls',
			executable='net_servo',
			name='net',
		)
		])

def main(args=None):
	generate_launch_descritpion()

if __name__=="__main__":
	main()
