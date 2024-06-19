#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

class CamNode : public rclcpp::Node {
public: 
    CamNode() : Node("cam_node"), frame_count_(0), minimum_radius_(15), find_goal_(true), cam_mode_(true), total_lines_(0), x_button_(0) {
        // Create publisher and subscriber
        cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);
        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CamNode::callback_read_joy, this, std::placeholders::_1));
        
        // Initialize video capture
        cap_.open(0, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        // HoughLinesP parameters
        rho_ = 1; 
        theta_ = CV_PI / 180;
        threshold_ = 75;
        min_line_length_ = 50;
        max_line_gap_ = 30;
        
        // Create timer to read images
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CamNode::callback_read_image, this));
        RCLCPP_INFO(this->get_logger(), "Video Detection has Started, press X button to switch detection");
    }

private:
    void callback_read_joy(const sensor_msgs::msg::Joy::SharedPtr button) {
        x_button_ = button->buttons[3];
    }

    void callback_read_image() {
        if (x_button_ == 1) {
            cam_mode_ = !cam_mode_;
            x_button_ = 0; // Reset the button state to avoid repeated toggling
            RCLCPP_INFO(this->get_logger(), "Cam Mode has been Switched");
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not open video source.");
            return;
        }

        cv::Mat frame, goal_frame;
        cap_ >> frame;
        cap_ >> goal_frame;

        if (frame.empty() || goal_frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not read frame.");
            return;
        }

        if (cam_mode_) {
            handle_balloon_detection(frame);
        } else {
            handle_goal_detection(goal_frame);
        }

        if (cv::waitKey(1) == 27) {
            rclcpp::shutdown();
        }
    }

    void handle_balloon_detection(cv::Mat& frame) {
        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        cv::Scalar lower_bound = cv::Scalar(120, 40, 30);
        cv::Scalar upper_bound = cv::Scalar(150, 255, 255);
        cv::Mat mask;
        cv::inRange(hsv_frame, lower_bound, upper_bound, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Point2f largest_center;
        float largest_radius = 0;
        for (const auto& contour : contours) {
            float radius;
            cv::Point2f center;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius > largest_radius) {
                largest_radius = radius;
                largest_center = center;
            }
        }

        if (largest_radius >= minimum_radius_ && largest_radius <= maximum_radius_) {
            cv::circle(frame, largest_center, static_cast<int>(largest_radius), cv::Scalar(0, 255, 0), 2);
            publish_coordinates(largest_center);
        }
    }

    void handle_goal_detection(cv::Mat& frame) {
        cv::Mat edges;
        cv::Canny(frame, edges, 50, 150);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, rho_, theta_, threshold_, min_line_length_, max_line_gap_);

        std::vector<cv::Point> midpoints;
        for (const auto& line : lines) {
            int mid_x = (line[0] + line[2]) / 2;
            int mid_y = (line[1] + line[3]) / 2;
            midpoints.push_back(cv::Point(mid_x, mid_y));
        }

        if (!midpoints.empty()) {
            auto [min_x, max_x] = std::minmax_element(midpoints.begin(), midpoints.end(), [](const cv::Point& p1, const cv::Point& p2) { return p1.x < p2.x; });
            auto [min_y, max_y] = std::minmax_element(midpoints.begin(), midpoints.end(), [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; });
            cv::Point center = cv::Point((min_x->x + max_x->x) / 2, (min_y->y + max_y->y) / 2);
            publish_coordinates(center);
        }
    }

    void publish_coordinates(const cv::Point& center) {
        auto msg = blimp_interfaces::msg::CameraCoord();
        msg.position = {center.x, center.y};
        cam_data_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Coordinates: X: %d, Y: %d", center.x, center.y);
    }

    // Member variables
    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    int frame_count_;
    int minimum_radius_;
    int maximum_radius_ = 300;
    bool cam_mode_;
    int counter_timer_;
    int x_button_;
    bool find_goal_;
    float rho_;
    float theta_;
    int threshold_;
    int min_line_length_;
    int max_line_gap_;
    int total_lines_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
