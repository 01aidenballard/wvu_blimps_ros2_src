#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "blimp_interfaces/msg/baro_data.hpp"
#include <vector>

class BalloonPI : public rclcpp::Node {
public:
    BalloonPI()
    : Node("balloon_pi"),
      coord_{320, 240}, x_int_error_(0.0), y_int_error_(0.0) {
        // Declare ROS parameters
        this->declare_parameter<double>("kpx", 0.0);
        this->declare_parameter<double>("kix", 0.0);
        this->declare_parameter<double>("kpy", 0.0);
        this->declare_parameter<double>("kiy", 0.0);
        this->declare_parameter<int>("x_goal", 320);
        this->declare_parameter<int>("y_goal", 240);
        this->declare_parameter<double>("kpb", 0.0);
        coord_ = {0, 0};
        coord_old = {1, 1};
        // Retrieve ROS parameters
        x_goal_ = this->get_parameter("x_goal").as_int();
        y_goal_ = this->get_parameter("y_goal").as_int();
        kpx_ = this->get_parameter("kpx").as_double();
        kix_ = this->get_parameter("kix").as_double();
        kpy_ = this->get_parameter("kpy").as_double();
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

        RCLCPP_INFO(this->get_logger(), "Started pi control for balloon detection.");
    }

private:
    void callback_camera_data(const blimp_interfaces::msg::CameraCoord::SharedPtr msg) {
        coord_old = coord_;
        coord_[0] = msg->position[0];
        coord_[1] = msg->position[1];
        
        
    }

    void callback_baro(const blimp_interfaces::msg::BaroData::SharedPtr msg) {
        if (coord_old[1] != coord_[1] && coord_old[0] != coord_[0]) {
            y_goal_ = msg->height;

            x_error = x_goal_ - coord_[0];
            y_error = y_goal_ - msg->height;

            x_int_error_ += x_error;
            // y_int_error_ += y_error;

            LR_input = x_error * kpx_ + x_int_error_ * kix_;
            UD_input = y_error * kpb_;


        } else {
            y_goal_ =  this->get_parameter("y_goal").as_int();

            x_error = x_goal_ - coord_[0];
            y_error = y_goal_ - coord_[1];

            x_int_error_ += x_error;
            y_int_error_ += y_error;

            LR_input = x_error * kpx_ + x_int_error_ * kix_;
            UD_input = y_error * kpy_ + y_int_error_ * kiy_;
        }
        
    }

    void callback_pi_control_balloon() {
        auto msg2 = blimp_interfaces::msg::CartCoord();

        

        

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
    int x_goal_, y_goal_;
    double kpx_, kix_, kpy_, kiy_, kpb_;
    
    // Control-related variables
    std::vector<int> coord_;
    std::vector<int> coord_old;
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