#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/srv/detection.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "opencv2/opencv.hpp"
#include "shared_frame.h"
#include <vector>
#include <memory>
#include <unordered_map>
//#include <mutex>

using std::placeholders::_1;
using std::placeholders::_2;

class BalloonDetectionServerNode : public rclcpp::Node {
    public:
        BalloonDetectionServerNode() : Node("balloon_detection_server") {
            server_ = this->create_service<blimp_interfaces::srv::Detection>(
                "balloon_detection", 
                std::bind(&BalloonDetectionServerNode::callback_balloon_detect, 
                this, _1, _2));

            // creates publisher, publishing on the topic "cam_data"
            cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);
            
            // setting up variables for balloon detection
            purple_lower_bound = cv::Scalar(120, 40, 30);
            purple_upper_bound = cv::Scalar(150, 255, 255);
            green_lower_bound = cv::Scalar(41, 80, 80);
            green_upper_bound = cv::Scalar(56, 255, 255);

            RCLCPP_INFO(this->get_logger(), "Balloon Detection Server has been started!");
        }

        const int minimum_radius = 15;
        const int maximum_radius = 300;
        cv::Scalar purple_lower_bound;
        cv::Scalar purple_upper_bound;
        cv::Scalar green_lower_bound;
        cv::Scalar green_upper_bound;
        rclcpp::Service<blimp_interfaces::srv::Detection>::SharedPtr server_;
        rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;


    private:
        void callback_balloon_detect(const std::shared_ptr<blimp_interfaces::srv::Detection::Request> request,
        const std::shared_ptr<blimp_interfaces::srv::Detection::Response> response) {
            // Creating HSV matrices to store the color filtering (getting frame from frame pointer)
            
            cv::Mat hsv_frame;
            //std::unique_lock<std::mutex> lock(*frame_mutex);
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
            //lock.unlock();

            // Color filtering mask matrices, leaves the HSV values NOT THE DETECTED COLOR
            cv::Mat purple_mask, green_mask;
            cv::inRange(hsv_frame, purple_lower_bound, purple_upper_bound, purple_mask);
            cv::inRange(hsv_frame, green_lower_bound, green_upper_bound, green_mask);

            std::vector<std::vector<cv::Point>> green_contours, purple_contours, all_contours;
            cv::findContours(purple_mask, purple_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::findContours(green_mask, green_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            all_contours.insert(all_contours.end(), purple_contours.begin(), purple_contours.end());
            all_contours.insert(all_contours.end(), green_contours.begin(), green_contours.end());

            cv::RotatedRect largest_contour;
            double largest_contour_area = 0;

            // Finding contour with the largest area
            for (const auto &contour : all_contours) {
                double contour_area = cv::contourArea(contour);
                if (contour_area  > largest_contour_area) {
                    largest_contour = cv::minAreaRect(contour);
                    largest_contour_area = contour_area;
                }
            }

            if (largest_contour.size.width != 0 && largest_contour.size.height != 0) {
                cv::Point2f center = largest_contour.center;
                int radius = std::max(largest_contour.size.width, largest_contour.size.height) / 2;
                if ((radius >= minimum_radius && radius <= maximum_radius) && (center.x >= 0 && center.x < frame.cols && center.y >= 0 && center.y < frame.rows)) {
                    RCLCPP_INFO(this->get_logger(), "CamNode - X: %f Y: %f", center.x, center.y);
                    auto msg = blimp_interfaces::msg::CameraCoord();
                    msg.position = {(long int) center.x, (long int) center.y};
                    cam_data_publisher_->publish(msg);

                    return;
                }
            }
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalloonDetectionServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
