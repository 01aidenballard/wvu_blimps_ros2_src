from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors_cpp',
            executable='modulated_light_camera',
            name='mod_cam',
            parameters=[
                {
                    "camera_width": 640,
                    "camera_height": 480,
                    "fps": 20,
                    "lower_freq": 5.0,
                    "upper_freq": 8.0
                }
            ]
        )
    ])
    def main(args=None):
        generate_launch_description()

    if __name__ == "__main__":
        main()
