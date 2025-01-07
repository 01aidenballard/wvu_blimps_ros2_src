from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_con',
            parameters=[{"autorepeat_rate": 10.0}]
        ),
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
        Node(
            package='sensors_cpp',
            executable='ES_control',
            name='Extremum',
            parameters=[
                {
                    "x_goal": 320,  # Goal x-coordinate
                    "y_goal": 240.0,  # Goal y-coordinate

                    # Parameters for vertical control
                    "hp_cutoff_freq_V": 0.05,    # High-pass filter cutoff frequency
                    "gain_V": 0.000001,        # Gain for the vertical extremum-seeking controller
                    "mod_signal_freq_V": 0.1,   # Frequency of modulation signal for vertical motor
                    "mod_signal_amplitude_V": 5.0,  # Amplitude of modulation signal for vertical motor

                    # Parameters for heading control (left/right motors)
                    "hp_cutoff_freq_LR": 0.12,  # High-pass filter cutoff frequency
                    "gain_LR": 0.0000001,       # Gain for the heading extremum-seeking controller
                    "mod_signal_freq_LR": 0.2, # Frequency of modulation signal for left/right motors
                    "mod_signal_amplitude_L": 0.8,  # Amplitude of modulation signal for left motor
                    "mod_signal_amplitude_R": 0.8   # Amplitude of modulation signal for right motor
                }
            ]
        ),
        Node(
            package='controls',
            name='esc_motor_driver',
            executable='esc_driver',
            parameters=[{"MAC": "68:6C:E6:73:04:62"}]
        ),
        Node(
            package='controls',
            executable='mode_switch',
            name='mode_switcher',
        )
    ])
