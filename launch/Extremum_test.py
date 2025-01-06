from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Outside dist packages
        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_con',
            parameters = [{"autorepeat_rate": 10.0}]
        ),
        # Manual Control Package Executable
        Node(
            package='manual_control',
            executable='joy_to_esc',
            name='joy_to_esc',
            parameters=[
                {
                    "Klm": 1.0,
                    "Krm": 1.0,
                }
            ]
        ),
        Node(
            package='sensors_cpp',
            executable='old_cam',
            name='old_cam_node',
        ),
        # Control Package Executables
        Node(
            package='sensors_cpp',
            executable='ES_control',
            name='Extremum',
            parameters=[
                {
                    # Original parameters for extremum seeking
                    "x_goal": 320,  # Goal x-coordinate
                    "y_goal": 240.0,  # Goal y-coordinate

                    # New parameters for process_motor_signals
                    # "hp_cutoff_freq": 0.05,               # High-pass filter cutoff frequency
                    # "gain": 0.0000000001,                     # Gain for the extremum-seeking controller
                    # "mod_signal_freq": 0.1,            # Frequency of modulation signal

                    "hp_cutoff_freq": 0.01,               # High-pass filter cutoff frequency
                    "gain": 0.000001,                     # Gain for the extremum-seeking controller
                    "mod_signal_freq": 0.05,            # Frequency of modulation signal

                    # Modulation signal amplitudes with multipliers
                    "mod_signal_amplitude_L": 100.0,  # Left motor amplitude with multiplier
                    "mod_signal_amplitude_R": 100.0,  # Right motor amplitude with multiplier
                    "mod_signal_amplitude_V": 200.0   # Vertical motor amplitude with multiplier
                }
            ]
        ),
        Node(
            package='controls',
            name='esc_motor_driver',
            executable='esc_driver',
            parameters=[
                {
                    "MAC": "68:6C:E6:73:04:62"
                }
            ]
        ),
        Node(
            package='controls',
            executable='mode_switch',
            name='mode_switcher',
        )
    ])
