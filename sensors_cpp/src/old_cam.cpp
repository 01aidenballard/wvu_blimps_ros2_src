#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "blimp_interfaces/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencv2/opencv.hpp" // Including OpenCV for camera operations
#include <thread>
#include <mutex>
#include <httplib.h> // Lightweight HTTP server
#include <vector>
#include <cmath>

class CamNode : public rclcpp::Node {
public:
    CamNode() : Node("cam_node"), current_frame_(), frame_mutex_() {
        // ROS2 Publishers and Subscribers
        cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);
        cam_flag_publisher_ = this->create_publisher<blimp_interfaces::msg::Bool>("cam_flag", 3);

        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&CamNode::callback_read_joy, this, std::placeholders::_1));

        subscriber_net_servo = this->create_subscription<blimp_interfaces::msg::Bool>(
            "net_flag", 10, std::bind(&CamNode::callback_read_net, this, std::placeholders::_1));

        // Initialize OpenCV video capture
        cap_ = cv::VideoCapture(0, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640); // 640
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 480

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            rclcpp::shutdown();
        }

        // Start Flask server thread
        server_thread_ = std::thread(&CamNode::startFlaskServer, this);

        // Start timer for periodic image capture
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), std::bind(&CamNode::callback_read_image, this));

        RCLCPP_INFO(this->get_logger(), "Video Detection has Started, press w to switch detection");
    }

    ~CamNode() {
        cap_.release();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    void releaseCapture() {
        cap_.release();
    }

    void callback_read_joy(const sensor_msgs::msg::Joy::SharedPtr button) {
        x_button = button->buttons[3];
    }

    void callback_read_net(const blimp_interfaces::msg::Bool &msg) {
        net_flag = msg.flag;
        RCLCPP_INFO(this->get_logger(), "Received net_flag: %s", net_flag ? "true" : "false");
    }

void callback_read_image() {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Error: Could not read frame.");
        return;
    }

    // HSV processing and contour detection
    cv::Mat hsv_frame, mask_goal;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

    // Define HSV range for red color detection
    cv::Scalar lower_red(0, 81, 201);   // Adjust these values based on your red color range
    cv::Scalar upper_red(179, 200, 255); // Adjust these values based on your red color range
    cv::inRange(hsv_frame, lower_red, upper_red, mask_goal);

    if (cam_mode && !net_flag) {
        goal_flag = false;

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_goal, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::RotatedRect largest_contour;
        double largest_contour_area = 0;

        for (const auto &contour : contours) {
            double contour_area = cv::contourArea(contour);
            if (contour_area > largest_contour_area) {
                largest_contour = cv::minAreaRect(contour);
                largest_contour_area = contour_area;
            }
        }

        if (largest_contour.size.width != 0 && largest_contour.size.height != 0) {
            cv::Point2f center = largest_contour.center;
            int radius = std::max(largest_contour.size.width, largest_contour.size.height) / 2;

            if (radius >= minimum_radius && radius <= maximum_radius) {
                if (center.x >= 0 && center.x < frame.cols && center.y >= 0 && center.y < frame.rows) {
                    detected_coords.push_back(center);
                    cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
                }
            }
        }

        total_x = 0;
        total_y = 0;
        for (const auto &coord : detected_coords) {
            total_x += coord.x;
            total_y += coord.y;
        }

        if (!detected_coords.empty()) {
            auto msg = blimp_interfaces::msg::CameraCoord();
            avg_x = std::round(total_x / detected_coords.size());
            avg_y = std::round(total_y / detected_coords.size());
            msg.position = {avg_x, avg_y};
            cam_data_publisher_->publish(msg);

            blimp_interfaces::msg::Bool flag_msg;
            flag_msg.flag = goal_flag;
            cam_flag_publisher_->publish(flag_msg);
        }

        detected_coords.clear();
    }

    // Optional: Highlight the detected regions in the frame
    cv::Mat masked_frame;
    cv::bitwise_and(frame, frame, masked_frame, mask_goal);

    // Optional: Combine the masked and unmasked frames side-by-side
    cv::Mat combined_frame;
    cv::hconcat(frame, masked_frame, combined_frame);

    // Update current_frame_ with the combined frame for streaming
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        current_frame_ = combined_frame.clone(); // Stream the combined frame (both masked and unmasked)
    }
}


private:
    cv::VideoCapture cap_;
    cv::Mat current_frame_;
    std::mutex frame_mutex_;
    std::thread server_thread_;

    int x_button = 0;
    bool cam_mode = true;
    bool net_flag = false;
    bool goal_flag = false;

    float total_x = 0, total_y = 0;
    int avg_x = 0, avg_y = 0;
    const int minimum_radius = 2;
    const int maximum_radius = 50;

    std::vector<cv::Point2f> detected_coords;

    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;
    rclcpp::Publisher<blimp_interfaces::msg::Bool>::SharedPtr cam_flag_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    rclcpp::Subscription<blimp_interfaces::msg::Bool>::SharedPtr subscriber_net_servo;
    rclcpp::TimerBase::SharedPtr timer_;

    void startFlaskServer() {
        httplib::Server server;

        server.Get("/video_feed", [this](const httplib::Request&, httplib::Response& res) {
            res.set_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [this](size_t /*offset*/, httplib::DataSink &sink) -> bool {
                    while (true) {
                        std::vector<uchar> buffer;
                        {
                            std::lock_guard<std::mutex> lock(frame_mutex_);
                            if (!current_frame_.empty()) {
                                cv::imencode(".jpg", current_frame_, buffer);
                            } else {
                                continue;
                            }
                        }

                        std::string frame_boundary = "--frame\r\n"
                                                     "Content-Type: image/jpeg\r\n\r\n";
                        sink.write(frame_boundary.c_str(), frame_boundary.size());
                        sink.write(reinterpret_cast<const char *>(buffer.data()), buffer.size());
                        sink.write("\r\n", 2);

                        std::this_thread::sleep_for(std::chrono::milliseconds(33));
                    }
                    return true;
                });
        });

        server.listen("0.0.0.0", 5000);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
