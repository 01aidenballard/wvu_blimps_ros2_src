from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
        # Lauching Xbox Controller
		Node(
			package='joy', # Package the node is in 
			executable='game_controller_node', # Executable found in setup.py 
			name='joy_con', # Name can be whatever you want
        ),

        Node(
			package ='sensors',
			executable='LED_modulation',
			name='LED',
		),
	])
def main(args=None):
	generate_launch_description()

if __name__ == "__main__":
	main()