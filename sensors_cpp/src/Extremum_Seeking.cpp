// #include "rclcpp/rclcpp.hpp"
// #include "blimp_interfaces/msg/cart_coord.hpp"
// #include "blimp_interfaces/msg/camera_coord.hpp"
// #include "blimp_interfaces/msg/esc_input.hpp"
// #include <vector>
// #include <cmath>
// #include <limits>
// #include <sstream>
// #include <chrono>

// class ExtremumSeekingEscInput : public rclcpp::Node {
// public:
//     ExtremumSeekingEscInput() 
//     : Node("extremum_seeking_esc_input"), 
//       coord_old_(std::numeric_limits<int>::quiet_NaN()), 
//       coord_{320, 240},
//       obj_function_{2,0},
//       esc_message_left{2,0},
//       esc_message_right{2,0},
//       esc_message_vert{2,0},
//       hp_out_L{2,0},
//       int_L{2,0},
//       hp_out_R{2,0},
//       int_R{2,0},
//       left_motor_input{2,0},
//       x_goal_(this->declare_parameter<int>("x_goal", 320)), 
//       y_goal_(this->declare_parameter<double>("y_goal", 240)), 
//       cam_message_{2, 0},
//       counter_(0),
//       counter2_(0),
//       counter3_(0),
//       hp_counter(0),
//       same_cam_msg_(false), 
//       PWM_initial_Left_(0),
//       PWM_initial_Right_(0),
//       total_time_(0),
//       timeout_duration_(std::chrono::seconds(5)),  
//       last_callback_time_(this->now()) 
//     {
//         // Subscribing to Camera
//         subscriber_camera_ = this->create_subscription<blimp_interfaces::msg::CameraCoord>(
//             "cam_data", 3, 
//             std::bind(&ExtremumSeekingEscInput::callback_cam_data, this, std::placeholders::_1)
//         );

//         // Publisher for ESC inputs
//         publisher_ = this->create_publisher<blimp_interfaces::msg::EscInput>("ESC_extremum_seeking_input", 10);

//         // Initialize the timer for checking timeout
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1), 
//             std::bind(&ExtremumSeekingEscInput::check_timeout, this)
//         );
//     }

// private:
//     void callback_cam_data(const blimp_interfaces::msg::CameraCoord::SharedPtr msg) {
//         // Update callback timestamp
//         last_callback_time_ = this->now();
//         time(&start_);
//         total_time_ = time(&start_);

//         coord_[0] = msg->position[0];
//         coord_[1] = msg->position[1]; 

//         int x_error = x_goal_ - coord_[0]; // x_error
//         //int y_error = coord_[1] - y_goal_; // y_error
//         //double x_output = coord_[0];

//         // Objective function
//         //obj_function_ = -1 * pow(x_error, 2);

//         // Compare current and previous camera messages
//         if (counter_ == 0) {
//             cam_message_[0] = coord_[0];
//             obj_function_[0] = (1 * pow(x_error, 2))/500;
//             counter_ = 1;
//         } else if (counter_ == 1) {
//             cam_message_[1] = coord_[0];
//             obj_function_[1] = (1 * pow(x_error, 2))/500;
//             counter_ = 2;
//         } else if (counter_ == 2) {
//             same_cam_msg_ = (cam_message_[0] == cam_message_[1]);
//             cam_message_[0] = cam_message_[1];
//             obj_function_[0] = obj_function_[1];
//             counter_ = 1;
//         }

//         //RCLCPP_INFO(this->get_logger(), "obj_function: %f",obj_function_[0]);

//         // Process motor signals
//         process_motor_signals();

//         // For Single-direction ESC 1100 is 0 Thrust
//         // Will still want up and down to be bidrectional ESC
//         // Update ESC input message
//         auto msg2 = blimp_interfaces::msg::EscInput();
//         if (!std::isnan(Left_Motor_) && !std::isnan(Right_Motor_) && !same_cam_msg_) {
//             msg2.esc_pins = {5, 6, 13};
//             msg2.pwm_l = Left_Motor_;
//             msg2.pwm_r = Right_Motor_;
//             msg2.pwm_d = 1500;
//         } else {
//             msg2.esc_pins = {5, 6, 13};
//             msg2.pwm_l = 1100;
//             msg2.pwm_r = 1100;
//             msg2.pwm_d = 1500;
//         }

//         if (counter2_ == 0) {
//             esc_message_left[0] = msg2.pwm_l;
//             esc_message_right[0] = msg2.pwm_r;
//             esc_message_vert[0] = msg2.pwm_d;
//             counter2_ = 1;
//         } else if (counter2_ == 1) {
//             esc_message_left[1] = msg2.pwm_l;
//             esc_message_right[1] = msg2.pwm_r;
//             esc_message_vert[1] = msg2.pwm_d;
//             counter2_ = 2;
//         } else if (counter2_ == 2) {
//             esc_message_left[0] = esc_message_left[1];
//             esc_message_right[0] = esc_message_right[1];
//             esc_message_vert[0] = esc_message_vert[1];
//             counter2_ = 1;
//         }
//         //RCLCPP_INFO(this->get_logger(), "esc_past: %f esc_old: %f",esc_message_left[0],esc_message_left[1]);
//         //RCLCPP_INFO(this->get_logger(), "counter 2: %d",counter2_);
//         publisher_->publish(msg2);
//         time(&finish_);
//         Ts_ = difftime(finish_, start_);
//     }

//     void check_timeout() {
//         // Check if the last callback exceeded the timeout duration
//         if ((this->now() - last_callback_time_) > timeout_duration_) {
//             //RCLCPP_WARN(this->get_logger(), "No camera callback received within the timeout period.");
//             // Reset ESC inputs to default
//             auto msg2 = blimp_interfaces::msg::EscInput();
//             msg2.esc_pins = {5, 6, 13};
//             msg2.pwm_l = 1100;
//             msg2.pwm_r = 1100;
//             msg2.pwm_d = 1500;
//             publisher_->publish(msg2);
//             hp_counter = 0;

//             if (counter3_ == 0) {
//                 esc_message_left[0] = esc_message_left[1]; 
//                 esc_message_right[0] = esc_message_right[1];
//                 esc_message_vert[0] = esc_message_vert[1];
//                 esc_message_left[1] = 1100; 
//                 esc_message_right[1] = 1100;
//                 esc_message_vert[1] = 1500;
//                 counter3_ = 1;
//             } else if (counter3_ == 1) {
//                 esc_message_left[0] = 1100; 
//                 esc_message_right[0] = 1100;
//                 esc_message_vert[0] = 1500;
//             }

//             //RCLCPP_INFO(this->get_logger(), "esc_past: %f esc_old: %f",esc_message_left[0],esc_message_left[1]);
//         }

//     }

//     // void process_motor_signals() {
//     //     // Processing motor signals for both left and right motors
//     //     // Lets Define Some Variables
//     //     // The system Dynamics Give us the output which is the camera coordinates (we have this collected for current time step and the previous time step)
//     //     // The objective function gives us our cost for our movement (also have this collected for the current and previous time steps)
//     //     // We then Need To Filter the Objective Funcction Output Through a Discrete Time High Pass Filter
//     //     // Now We also need to collect current and previous time steps of this filter as well.
        
//     //     // Left Motor //
//     //     // High Pass Filter
//     //     //Ts_ = .05;
//     //     double hp_cutoff_freq_L = 0.1*0.05*3.14*2;
//     //     double hp_alpha_L = (2-Ts_*hp_cutoff_freq_L)/(2+Ts_*hp_cutoff_freq_L);
//     //     //RCLCPP_INFO(this->get_logger(), "hp_alpha %f",hp_alpha_L);
//     //     // Demodulation Signal
//     //     double demod_amp_L = 50; // 50
//     //     double demod_force_freq_L = 0.05*3.14*2; //.05
//     //     double demod_signal_L = demod_amp_L*(sin(demod_force_freq_L*total_time_+3.14/3));  
//     //     //RCLCPP_INFO(this->get_logger(), "demond signal: %f",demod_signal_L);
//     //     // Mod Signal
//     //     double mod_amp_L = 100;
//     //     double mod_force_freq_L = 0.05*3.14*2;
//     //     double mod_signal_L = mod_amp_L*(sin(mod_force_freq_L*total_time_+3.14/2)); 
//     //     //RCLCPP_INFO(this->get_logger(), "mod signal: %f",mod_signal_L);  
//     //     // Gain
//     //     double gain_L = 0.0001; //.0001
//     //     // Integrator in Counter;  

//     //      if (hp_counter == 0) {
//     //         Left_Motor_ = 1100 + mod_signal_L;
//     //         hp_out_L[0] = (hp_alpha_L*(obj_function_[0]))*demod_signal_L; // here we have no initial values
//     //         int_L[0] = gain_L*hp_out_L[0]*Ts_;
//     //         hp_counter = 1;
//     //     } else if (hp_counter == 1) {
//     //         Left_Motor_ = 1100 + int_L[0] + mod_signal_L;
//     //         hp_out_L[1] = hp_alpha_L*(hp_out_L[0]+obj_function_[1]-obj_function_[0]);
//     //         int_L[1] = int_L[0] - gain_L*hp_out_L[0];
//     //         hp_counter = 2;
//     //     } else if (hp_counter == 2) {
//     //         Left_Motor_ = 1100 + int_L[1] + mod_signal_L;
//     //         hp_out_L[0] = hp_out_L[1];
//     //         int_L[0] = int_L[1];
//     //         hp_counter = 1;
//     //     }
//     //     RCLCPP_INFO(this->get_logger(), "Left Motor: %f",Left_Motor_);
//     //     //RCLCPP_INFO(this->get_logger(), "hp_past: %f hp_old: %f",hp_out_L[0],hp_out_L[1]);

//     //     // Limiting PWM Signal to +- 200
//     //     if (Left_Motor_ > 1500) {
//     //         Left_Motor_ = 1500;
//     //     } else if (Left_Motor_ < 1100) {
//     //         Left_Motor_ = 1100;
//     //     }
        
        
//     //     // SImple Filtering
//     //     // Left Motor signal
//     //     //double mod_signal_Left = 110 * sin(0.4 * total_time_);
//     //     //double demod_signal_Left = 340 * sin(0.4 * total_time_);
//     //     //double High_Pass_Filter_Left = (1) / (1 - 0.35);
//     //     //double Low_Pass_Filter_Left = (1) / (1 - 0.39);
//     //     //double gain_Left = 1;

//     //     // Right Motor signal
//     //     double mod_signal_Right = 110 * cos(0.9 * total_time_);
//     //     double demod_signal_Right = 340 * cos(0.9 * total_time_);
//     //     double High_Pass_Filter_Right = (1) / (1 - 0.65);
//     //     double Low_Pass_Filter_Right = (1) / (1 - 0.7);
//     //     double gain_Right = 1;

//     //     // Update motor PWM values
//     //     double integrator = (Ts_ / 2) * (1) / (1);
//     //     //Left_Motor_ = 1100 + PWM_initial_Left_ + mod_signal_Left;
//     //     //RCLCPP_INFO(this->get_logger(), "Left Motor: %f",Left_Motor_);
//     //     Right_Motor_ = 1100 + PWM_initial_Right_ + mod_signal_Right;
//     //     //PWM_initial_Left_ = High_Pass_Filter_Left * demod_signal_Left * Low_Pass_Filter_Left * gain_Left * integrator;
//     //     PWM_initial_Right_ = High_Pass_Filter_Right * demod_signal_Right * Low_Pass_Filter_Right * gain_Right * integrator;
//     // }
// void process_motor_signals() {
//     // Left Motor //
//     double hp_cutoff_freq_L = 0.1 * 0.05 * 3.14 * 2;
//     double hp_alpha_L = (2 - Ts_ * hp_cutoff_freq_L) / (2 + Ts_ * hp_cutoff_freq_L);
//     double demod_amp_L = 50;  // 50
//     double demod_force_freq_L = 0.05 * 3.14 * 2;  // 0.05
//     double demod_signal_L = demod_amp_L * (sin(demod_force_freq_L * total_time_ + 3.14 / 3));
//     double mod_amp_L = 100;
//     double mod_force_freq_L = 0.05 * 3.14 * 2;
//     double mod_signal_L = mod_amp_L * (sin(mod_force_freq_L * total_time_ + 3.14 / 2));
//     double gain_L = 0.0001;

//     if (hp_counter == 0) {
//         Left_Motor_ = 1100 + mod_signal_L;
//         hp_out_L[0] = (hp_alpha_L * obj_function_[0]) * demod_signal_L;
//         int_L[0] = gain_L * hp_out_L[0] * Ts_;
//         hp_counter = 1;
//     } else if (hp_counter == 1) {
//         Left_Motor_ = 1100 + int_L[0] + mod_signal_L;
//         hp_out_L[1] = hp_alpha_L * (hp_out_L[0] + obj_function_[1] - obj_function_[0]);
//         int_L[1] = int_L[0] - gain_L * hp_out_L[0];
//         hp_counter = 2;
//     } else if (hp_counter == 2) {
//         Left_Motor_ = 1100 + int_L[1] + mod_signal_L;
//         hp_out_L[0] = hp_out_L[1];
//         int_L[0] = int_L[1];
//         hp_counter = 1;
//     }

//     if (Left_Motor_ > 1500) {
//         Left_Motor_ = 1500;
//     } else if (Left_Motor_ < 1100) {
//         Left_Motor_ = 1100;
//     }

//     // Right Motor //
//     double hp_cutoff_freq_R = 0.1 * 0.05 * 3.14 * 2;
//     double hp_alpha_R = (2 - Ts_ * hp_cutoff_freq_R) / (2 + Ts_ * hp_cutoff_freq_R);
//     double demod_amp_R = 50;  // Same values as left motor
//     double demod_force_freq_R = 0.05 * 3.14 * 2;
//     double demod_signal_R = demod_amp_R * (cos(demod_force_freq_R * total_time_ + 3.14 / 3));
//     double mod_amp_R = 100;
//     double mod_force_freq_R = 0.05 * 3.14 * 2;
//     double mod_signal_R = mod_amp_R * (cos(mod_force_freq_R * total_time_ + 3.14 / 2));
//     double gain_R = 0.0001;

//     if (hp_counter == 0) {
//         Right_Motor_ = 1100 + mod_signal_R;
//         hp_out_R[0] = (hp_alpha_R * obj_function_[0]) * demod_signal_R;
//         int_R[0] = gain_R * hp_out_R[0] * Ts_;
//     } else if (hp_counter == 1) {
//         Right_Motor_ = 1100 + int_R[0] + mod_signal_R;
//         hp_out_R[1] = hp_alpha_R * (hp_out_R[0] + obj_function_[1] - obj_function_[0]);
//         int_R[1] = int_R[0] - gain_R * hp_out_R[0];
//     } else if (hp_counter == 2) {
//         Right_Motor_ = 1100 + int_R[1] + mod_signal_R;
//         hp_out_R[0] = hp_out_R[1];
//         int_R[0] = int_R[1];
//     }

//     if (Right_Motor_ > 1500) {
//         Right_Motor_ = 1500;
//     } else if (Right_Motor_ < 1100) {
//         Right_Motor_ = 1100;
//     }

//     RCLCPP_INFO(this->get_logger(), "Left Motor: %f, Right Motor: %f", Left_Motor_, Right_Motor_);
// }

//     // Member variables
//     int coord_old_;
//     std::vector<int> coord_;
//     std::vector<double> obj_function_;
//     std::vector<double> esc_message_left;
//     std::vector<double> esc_message_right;
//     std::vector<double> esc_message_vert;
//     std::vector<double> hp_out_L;
//     std::vector<double> int_L;
//     std::vector<double> hp_out_R;
//     std::vector<double> int_R;
//     std::vector<double> left_motor_input;
//     double x_goal_, y_goal_;
//     std::vector<int> cam_message_;
//     int counter_;
//     int counter2_;
//     int counter3_;
//     int hp_counter;
//     bool same_cam_msg_;
//     double PWM_initial_Left_, PWM_initial_Right_, total_time_;
//     double Left_Motor_, Right_Motor_;
//     double Ts_;
//     time_t start_, finish_;
    
//     rclcpp::Duration timeout_duration_;
//     rclcpp::Time last_callback_time_;
    
//     rclcpp::Subscription<blimp_interfaces::msg::CameraCoord>::SharedPtr subscriber_camera_;
//     rclcpp::Publisher<blimp_interfaces::msg::EscInput>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ExtremumSeekingEscInput>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

// Workinng Take 2 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #include "rclcpp/rclcpp.hpp"
// #include "blimp_interfaces/msg/cart_coord.hpp"
// #include "blimp_interfaces/msg/camera_coord.hpp"
// #include "blimp_interfaces/msg/esc_input.hpp"
// #include <vector>
// #include <cmath>
// #include <limits>
// #include <sstream>
// #include <chrono>

// class ExtremumSeekingEscInput : public rclcpp::Node {
// public:
//     ExtremumSeekingEscInput()
//     : Node("extremum_seeking_esc_input"),
//       coord_old_(std::numeric_limits<int>::quiet_NaN()),
//       coord_{320, 240},
//       obj_function_x_{2, 0},
//       obj_function_y_{2, 0},
//       esc_message_left_{2, 0},
//       esc_message_right_{2, 0},
//       esc_message_vert_{2, 0},
//       hp_out_L_{2, 0},
//       int_L_{2, 0},
//       hp_out_R_{2, 0},
//       int_R_{2, 0},
//       hp_out_V_{2, 0},
//       int_V_{2, 0},
//       Left_Motor_(1050),
//       Right_Motor_(1050),
//       Vertical_Motor_(1050),
//       x_goal_(this->declare_parameter<int>("x_goal", 320)),
//       y_goal_(this->declare_parameter<double>("y_goal", 240)),
//       cam_message_{2, 0},
//       counter_(0),
//       counter2_(0),
//       counter3_(0),
//       hp_counter_(0),
//       same_cam_msg_(false),
//       PWM_initial_Left_(0),
//       PWM_initial_Right_(0),
//       total_time_(0),
//       timeout_duration_(std::chrono::seconds(5)),
//       last_callback_time_(this->now())
//     {
//         subscriber_camera_ = this->create_subscription<blimp_interfaces::msg::CameraCoord>(
//             "cam_data", 3, 
//             std::bind(&ExtremumSeekingEscInput::callback_cam_data, this, std::placeholders::_1)
//         );

//         publisher_ = this->create_publisher<blimp_interfaces::msg::EscInput>("ESC_extremum_seeking_input", 10);

//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1), 
//             std::bind(&ExtremumSeekingEscInput::check_timeout, this)
//         );
//     }

// private:
//     void callback_cam_data(const blimp_interfaces::msg::CameraCoord::SharedPtr msg) {
//         last_callback_time_ = this->now();
//         time(&start_);
//         total_time_ = time(&start_);

//         coord_[0] = msg->position[0];
//         coord_[1] = msg->position[1];

//         int x_error = x_goal_ - coord_[0];
//         int y_error = coord_[1] - y_goal_;

//         if (counter_ == 0) {
//             cam_message_[0] = coord_[0];
//             obj_function_x_[0] = (1 * pow(x_error, 2)) / 500;
//             obj_function_y_[0] = (1 * pow(y_error, 2)) / 500;
//             counter_ = 1;
//         } else if (counter_ == 1) {
//             cam_message_[1] = coord_[0];
//             obj_function_x_[1] = (1 * pow(x_error, 2)) / 500;
//             obj_function_y_[1] = (1 * pow(y_error, 2)) / 500;
//             counter_ = 2;
//         } else if (counter_ == 2) {
//             same_cam_msg_ = (cam_message_[0] == cam_message_[1]);
//             cam_message_[0] = cam_message_[1];
//             obj_function_x_[0] = obj_function_x_[1];
//             obj_function_y_[0] = obj_function_y_[1];
//             counter_ = 1;
//         }

//         process_motor_signals();

//         auto msg2 = blimp_interfaces::msg::EscInput();
//         if (!std::isnan(Left_Motor_) && !std::isnan(Right_Motor_) && !same_cam_msg_) {
//             msg2.esc_pins = {5, 6, 13};
//             //msg2.pwm_l = Left_Motor_;
//             //msg2.pwm_r = Right_Motor_;
// 	    msg2.pwm_l = 1050;
// 	    msg2.pwm_r = 1050;
//             msg2.pwm_d = Vertical_Motor_;
//         } else {
//             msg2.esc_pins = {5, 6, 13};
//             msg2.pwm_l = 1050;
//             msg2.pwm_r = 1050;
//             msg2.pwm_d = 1050;
//         }

//         if (counter2_ == 0) {
//             esc_message_left_[0] = msg2.pwm_l;
//             esc_message_right_[0] = msg2.pwm_r;
//             esc_message_vert_[0] = msg2.pwm_d;
//             counter2_ = 1;
//         } else if (counter2_ == 1) {
//             esc_message_left_[1] = msg2.pwm_l;
//             esc_message_right_[1] = msg2.pwm_r;
//             esc_message_vert_[1] = msg2.pwm_d;
//             counter2_ = 2;
//         } else if (counter2_ == 2) {
//             esc_message_left_[0] = esc_message_left_[1];
//             esc_message_right_[0] = esc_message_right_[1];
//             esc_message_vert_[0] = esc_message_vert_[1];
//             counter2_ = 1;
//         }

//         publisher_->publish(msg2);
//         time(&finish_);
//         Ts_ = difftime(finish_, start_);
//     }

//     void process_motor_signals() {
//         double hp_cutoff_freq = 0.1 * 0.05 * 3.14 * 2;
//         double hp_alpha = (2 - Ts_ * hp_cutoff_freq) / (2 + Ts_ * hp_cutoff_freq);
//         double gain = 0.0001;

//         // Left Motor
//         double mod_signal_L = 100 * sin(0.05 * 3.14 * 2 * total_time_ + 3.14 / 2);
//         if (hp_counter_ == 0) {
//             Left_Motor_ = 1050 + mod_signal_L;
//             hp_out_L_[0] = (hp_alpha * obj_function_x_[0]) * mod_signal_L;
//             int_L_[0] = gain * hp_out_L_[0] * Ts_;
//         } else {
//             Left_Motor_ = 1050 + int_L_[0] + mod_signal_L;
//             hp_out_L_[1] = hp_alpha * (hp_out_L_[0] + obj_function_x_[1] - obj_function_x_[0]);
//             int_L_[1] = int_L_[0] - gain * hp_out_L_[0];
//             hp_out_L_[0] = hp_out_L_[1];
//             int_L_[0] = int_L_[1];
//         }

//         if (Left_Motor_ > 1500) Left_Motor_ = 1500;
//         if (Left_Motor_ < 1050) Left_Motor_ = 1050;

//         // Right Motor
//         double mod_signal_R = 100 * cos(0.05 * 3.14 * 2 * total_time_ + 3.14 / 2);
//         if (hp_counter_ == 0) {
//             Right_Motor_ = 1050 + mod_signal_R;
//             hp_out_R_[0] = (hp_alpha * obj_function_x_[0]) * mod_signal_R;
//             int_R_[0] = gain * hp_out_R_[0] * Ts_;
//         } else {
//             Right_Motor_ = 1050 + int_R_[0] + mod_signal_R;
//             hp_out_R_[1] = hp_alpha * (hp_out_R_[0] + obj_function_x_[1] - obj_function_x_[0]);
//             int_R_[1] = int_R_[0] - gain * hp_out_R_[0];
//             hp_out_R_[0] = hp_out_R_[1];
//             int_R_[0] = int_R_[1];
//         }

//         if (Right_Motor_ > 1500) Right_Motor_ = 1500;
//         if (Right_Motor_ < 1050) Right_Motor_ = 1050;

//         // Vertical Motor
//         double mod_signal_V = 500 * sin(0.05 * 3.14 * 2 * total_time_ + 3.14 / 2);
//         if (hp_counter_ == 0) {
//             Vertical_Motor_ = 1050 + mod_signal_V;
//             hp_out_V_[0] = (hp_alpha * obj_function_y_[0]) * mod_signal_V;
//             int_V_[0] = gain * hp_out_V_[0] * Ts_;
//         } else {
//             Vertical_Motor_ = 1050 + int_V_[0] + mod_signal_V;
//             hp_out_V_[1] = hp_alpha * (hp_out_V_[0] + obj_function_y_[1] - obj_function_y_[0]);
//             int_V_[1] = int_V_[0] - gain * hp_out_V_[0];
//             hp_out_V_[0] = hp_out_V_[1];
//             int_V_[0] = int_V_[1];
//         }

//         if (Vertical_Motor_ > 1500) Vertical_Motor_ = 1500;
//         if (Vertical_Motor_ < 1050) Vertical_Motor_ = 1050;
//     }

//     void check_timeout() {
//         if ((this->now() - last_callback_time_) > timeout_duration_) {
//             auto msg = blimp_interfaces::msg::EscInput();
//             msg.esc_pins = {5, 6, 13};
//             msg.pwm_l = 1050;
//             msg.pwm_r = 1050;
//             msg.pwm_d = 1050;
//             publisher_->publish(msg);
//         }
//     }

//     int coord_old_;
//     std::vector<int> coord_;
//     std::vector<double> obj_function_x_, obj_function_y_;
//     std::vector<double> esc_message_left_, esc_message_right_, esc_message_vert_;
//     std::vector<double> hp_out_L_, int_L_, hp_out_R_, int_R_, hp_out_V_, int_V_;
//     double Left_Motor_, Right_Motor_, Vertical_Motor_;
//     double x_goal_, y_goal_;
//     std::vector<int> cam_message_;
//     int counter_, counter2_, counter3_, hp_counter_;
//     bool same_cam_msg_;
//     double PWM_initial_Left_, PWM_initial_Right_, total_time_;
//     double Ts_;
//     time_t start_, finish_;
//     rclcpp::Duration timeout_duration_;
//     rclcpp::Time last_callback_time_;
//     rclcpp::Subscription<blimp_interfaces::msg::CameraCoord>::SharedPtr subscriber_camera_;
//     rclcpp::Publisher<blimp_interfaces::msg::EscInput>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ExtremumSeekingEscInput>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "blimp_interfaces/msg/esc_input.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <sstream>
#include <chrono>

class ExtremumSeekingEscInput : public rclcpp::Node {
public:
    ExtremumSeekingEscInput()
    : Node("extremum_seeking_esc_input"),
      hp_cutoff_freq_(this->declare_parameter<double>("hp_cutoff_freq", 0.1)),
      gain_(this->declare_parameter<double>("gain", 0.0001)),
      mod_signal_freq_(this->declare_parameter<double>("mod_signal_freq", 0.05)),
      mod_signal_amplitude_L_(this->declare_parameter<double>("mod_signal_amplitude_L", 100.0)),
      mod_signal_amplitude_R_(this->declare_parameter<double>("mod_signal_amplitude_R", 100.0)),
      mod_signal_amplitude_V_(this->declare_parameter<double>("mod_signal_amplitude_V", 500.0)),
      coord_old_(std::numeric_limits<int>::quiet_NaN()),
      coord_{320, 240},
      obj_function_x_{2, 0},
      obj_function_y_{2, 0},
      esc_message_left_{2, 0},
      esc_message_right_{2, 0},
      esc_message_vert_{2, 0},
      hp_out_L_{2, 0},
      int_L_{2, 0},
      hp_out_R_{2, 0},
      int_R_{2, 0},
      hp_out_V_{2, 0},
      int_V_{2, 0},
      Left_Motor_(1050),
      Right_Motor_(1050),
      Vertical_Motor_(1050),
      x_goal_(this->declare_parameter<int>("x_goal", 320)),
      y_goal_(this->declare_parameter<double>("y_goal", 240)),
      cam_message_{2, 0},
      counter_(0),
      counter2_(0),
      counter3_(0),
      hp_counter_(0),
      same_cam_msg_(false),
      PWM_initial_Left_(0),
      PWM_initial_Right_(0),
      total_time_(0),
      timeout_duration_(std::chrono::seconds(5)),
      last_callback_time_(this->now())
    {
        subscriber_camera_ = this->create_subscription<blimp_interfaces::msg::CameraCoord>(
            "cam_data", 3, 
            std::bind(&ExtremumSeekingEscInput::callback_cam_data, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<blimp_interfaces::msg::EscInput>("ESC_extremum_seeking_input", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&ExtremumSeekingEscInput::check_timeout, this)
        );
    }

private:
    void process_motor_signals() {
        double hp_alpha = (2 - Ts_ * hp_cutoff_freq_) / (2 + Ts_ * hp_cutoff_freq_);

        // Process Vertical Motor dynamically
        double mod_signal_V = mod_signal_amplitude_V_ * sin(mod_signal_freq_ * 3.14 * 2 * total_time_ + 3.14 / 2);
        if (hp_counter_ == 0) {
            Vertical_Motor_ = 1050 + mod_signal_V;
            hp_out_V_[0] = (hp_alpha * obj_function_y_[0]) * mod_signal_V;
            int_V_[0] = gain_ * hp_out_V_[0] * Ts_;
        } else {
            Vertical_Motor_ = 1050 + int_V_[0] + mod_signal_V;
            hp_out_V_[1] = hp_alpha * (hp_out_V_[0] + obj_function_y_[1] - obj_function_y_[0]);
            int_V_[1] = int_V_[0] - gain_ * hp_out_V_[0];
            hp_out_V_[0] = hp_out_V_[1];
            int_V_[0] = int_V_[1];
        }

        if (Vertical_Motor_ > 1500) Vertical_Motor_ = 1500;
        if (Vertical_Motor_ < 1050) Vertical_Motor_ = 1050;

        // Publish the PWM signals
        auto msg2 = blimp_interfaces::msg::EscInput();
        if (!std::isnan(Left_Motor_) && !std::isnan(Right_Motor_) && !same_cam_msg_) {
            msg2.esc_pins = {5, 6, 13};
            // msg2.pwm_l = Left_Motor_; // Commented out
            // msg2.pwm_r = Right_Motor_; // Commented out
            msg2.pwm_l = 1050;  // Hardcoded for testing
            msg2.pwm_r = 1050;  // Hardcoded for testing
            msg2.pwm_d = Vertical_Motor_;
            RCLCPP_INFO(this->get_logger(), "PWM Values - Left: 1050, Right: 1050, Vertical: %f", Vertical_Motor_);
        } else {
            msg2.esc_pins = {5, 6, 13};
            msg2.pwm_l = 1050;
            msg2.pwm_r = 1050;
            msg2.pwm_d = 1050;
            RCLCPP_WARN(this->get_logger(), "Fallback PWM - Left: 1050, Right: 1050, Vertical: 1050");
        }
        publisher_->publish(msg2);
    }

    void callback_cam_data(const blimp_interfaces::msg::CameraCoord::SharedPtr msg) {
        last_callback_time_ = this->now();
        time(&start_);
        total_time_ = time(&start_);

        coord_[0] = msg->position[0];
        coord_[1] = msg->position[1];

        int x_error = x_goal_ - coord_[0];
        int y_error = coord_[1] - y_goal_;

        if (counter_ == 0) {
            cam_message_[0] = coord_[0];
            obj_function_x_[0] = (1 * pow(x_error, 2)) / 500;
            obj_function_y_[0] = (1 * pow(y_error, 2)) / 500;
            counter_ = 1;
        } else if (counter_ == 1) {
            cam_message_[1] = coord_[0];
            obj_function_x_[1] = (1 * pow(x_error, 2)) / 500;
            obj_function_y_[1] = (1 * pow(y_error, 2)) / 500;
            counter_ = 2;
        } else if (counter_ == 2) {
            same_cam_msg_ = (cam_message_[0] == cam_message_[1]);
            cam_message_[0] = cam_message_[1];
            obj_function_x_[0] = obj_function_x_[1];
            obj_function_y_[0] = obj_function_y_[1];
            counter_ = 1;
        }

        process_motor_signals();
    }

    void check_timeout() {
        if ((this->now() - last_callback_time_) > timeout_duration_) {
            auto msg = blimp_interfaces::msg::EscInput();
            msg.esc_pins = {5, 6, 13};
            msg.pwm_l = 1050;
            msg.pwm_r = 1050;
            msg.pwm_d = 1050;
            publisher_->publish(msg);
        }
    }

    double hp_cutoff_freq_, gain_;
    double mod_signal_freq_;
    double mod_signal_amplitude_L_, mod_signal_amplitude_R_, mod_signal_amplitude_V_;
    int coord_old_;
    std::vector<int> coord_;
    std::vector<double> obj_function_x_, obj_function_y_;
    std::vector<double> esc_message_left_, esc_message_right_, esc_message_vert_;
    std::vector<double> hp_out_L_, int_L_, hp_out_R_, int_R_, hp_out_V_, int_V_;
    double Left_Motor_, Right_Motor_, Vertical_Motor_;
    double x_goal_, y_goal_;
    std::vector<int> cam_message_;
    int counter_, counter2_, counter3_, hp_counter_;
    bool same_cam_msg_;
    double PWM_initial_Left_, PWM_initial_Right_, total_time_;
    double Ts_;
    time_t start_, finish_;
    rclcpp::Duration timeout_duration_;
    rclcpp::Time last_callback_time_;
    rclcpp::Subscription<blimp_interfaces::msg::CameraCoord>::SharedPtr subscriber_camera_;
    rclcpp::Publisher<blimp_interfaces::msg::EscInput>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtremumSeekingEscInput>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
