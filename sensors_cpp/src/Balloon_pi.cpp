//DONT EDIT THIS CODE TEST.CPP IS THE REPLACEMENT FOR THIS CODE
//%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "blimp_interfaces/msg/baro_data.hpp"
#include <vector>
#include <cmath>

class BalloonPI : public rclcpp::Node {
public:
    BalloonPI()
    : Node("balloon_pi"),
      coord_{320, 240}, x_int_error_(0.0), y_int_error_(0.0) {
        // Declare ROS parameters
	this->declare_parameter<double>("iheight",0.0);
        this->declare_parameter<double>("kpx", 0.0);
        this->declare_parameter<double>("kix", 0.0);
        this->declare_parameter<double>("kpyu", 0.0);
        this->declare_parameter<double>("kpyd", 0.0);
        this->declare_parameter<double>("kiy", 0.0);
        this->declare_parameter<int>("x_goal", 320);
        this->declare_parameter<double>("y_goal", 240);
        this->declare_parameter<double>("kpb", 0.0);
        coord_ = {0, 0};
        coord_old = {1, 1};
        // Retrieve ROS parameters
        x_goal_ = this->get_parameter("x_goal").as_int();
        y_goal_ = this->get_parameter("y_goal").as_double();

        kpx_ = this->get_parameter("kpx").as_double();
        kix_ = this->get_parameter("kix").as_double();
        kpyu_ = this->get_parameter("kpyu").as_double();
        kpyd_ = this->get_parameter("kpyd").as_double();
        kiy_ = this->get_parameter("kiy").as_double();
        kpb_ = this->get_parameter("kpb").as_double();


        // Setup subscriber
        subscriber_ = this->create_subscription<blimp_interfaces::msg::CameraCoord>(
            "cam_data", 3, std::bind(&BalloonPI::callback_camera_data, this, std::placeholders::_1));

        subscriber_baro = this->create_subscription<blimp_interfaces::msg::BaroData>(
            "barometer_data", 10, std::bind(&BalloonPI::callback_baro, this, std::placeholders::_1));
        // Setup publisher
        publisher_ = this->create_publisher<blimp_interfaces::msg::CartCoord>("balloon_input", 10);

        // Timer to repeatedly call callback_pi_control_balloon()
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&BalloonPI::callback_pi_control_balloon, this));
	    time(&start);
	    height_goal = this->get_parameter("iheight").as_double();
	    height_update = 0.0;
        RCLCPP_INFO(this->get_logger(), "Started pi control for balloon detection.");
    }

private:
    void callback_camera_data(const blimp_interfaces::msg::CameraCoord::SharedPtr msg) {
        coord_[0] = msg->position[0];
        coord_[1] = msg->position[1];
        time(&start);
	    height_goal = height;
        
    }

    void callback_baro(const blimp_interfaces::msg::BaroData::SharedPtr msg) {
        height = msg->height;
        
    }

    void callback_pi_control_balloon() {
        
	    time(&finish);
	    dt = difftime(finish, start);
        if (dt > 5) {
            x_error = x_goal_ - coord_[0];
            y_error = abs(height) - abs(height_goal);

            x_int_error_ += x_error;

            LR_input = x_error * kpx_ + x_int_error_ * kix_;
            UD_input = y_error * kpb_;
        } else {
            x_error = x_goal_ - coord_[0];
            y_error = coord_[1] -  y_goal_;

            x_int_error_ += x_error;
            y_int_error_ += y_error;

            LR_input = x_error * kpx_ + x_int_error_ * kix_;
            if (y_error < 0) {
                UD_input = y_error * kpyu_ + y_int_error_ * kiy_;
            } else {
                UD_input = y_error * kpyd_ + y_int_error_ * kiy_;
            }
           
        }


        auto msg2 = blimp_interfaces::msg::CartCoord();
	//RCLCPP_INFO(this->get_logger(), "dt: %f, height_goal: %f, y_error: %d", dt, height_goal, y_error);
        msg2.x = 0;
        msg2.y = 0;
        msg2.z  = UD_input;
        msg2.theta = 0;
        msg2.phi = 0;
        msg2.psy = LR_input;

        // Publish the control message
        publisher_->publish(msg2);
        //RCLCPP_INFO(this->get_logger(), "UD_accel: %f  LR_accel: %f", UD_input, LR_input);
    }

    // Node parameters
    int x_goal_;
    double kpx_, kix_, kpyu_, kiy_, kpb_, kpyd_, y_goal_;
    double height_goal, height_update;
    double dt;
    // Control-related variables
    std::vector<int> coord_;
    std::vector<int> coord_old;
    time_t start, finish;
    int x_error, y_error;
    double x_int_error_, y_int_error_;
    double LR_input;
    double UD_input;
    double height;

    // ROS communication interfaces
    rclcpp::Subscription<blimp_interfaces::msg::CameraCoord>::SharedPtr subscriber_;
    rclcpp::Publisher<blimp_interfaces::msg::CartCoord>::SharedPtr publisher_;
    rclcpp::Subscription<blimp_interfaces::msg::BaroData>::SharedPtr subscriber_baro;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalloonPI>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
