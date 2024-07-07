#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/srv/detection.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "opencv2/opencv.hpp"
#include "shared_frame.h"
#include <vector>
//#include <mutex>

using std::placeholders::_1;
using std::placeholders::_2;

class GoalDetectionServer : public rclcpp::Node {
    public:
        GoalDetectionServer() : Node("goal_detection_server") {
            server_ = this->create_service<blimp_interfaces::srv::Detection>(
                "goal_detection",
                std::bind(&GoalDetectionServer::callback_goal_detect,
                this, _1, _2));

            // creates publisher, publishing on the topic "cam_data"
            cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);

            // setting up variables for goal detection
            rho = 1;
            theta = CV_PI / 180;
            threshold = 75;
            low_threshold = 250;
            high_threshold = 300;
            yellow_lower_bound = cv::Scalar(28, 80, 120);
            yellow_upper_bound = cv::Scalar(36, 255, 255);
            //orange_lower_bound = cv::Scalar(1,120,50);
            //orange_upper_bound = cv::Scalar(12,255,255);

            RCLCPP_INFO(this->get_logger(), "Goal Detection Server has started.");
        }

    rclcpp::Service<blimp_interfaces::srv::Detection>::SharedPtr server_;
    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;
    int rho;
    int theta;
    int threshold;
    int min_line_length;
    int max_line_gap;
    int low_threshold;
    int high_threshold;
    cv::Scalar yellow_lower_bound;
    cv::Scalar yellow_upper_bound;
	//cv::Scalar orange_lower_bound;
	//cv::Scalar orange_upper_bound;

    private:
        void callback_goal_detect(const std::shared_ptr<blimp_interfaces::srv::Detection::Request> request,
        const std::shared_ptr<blimp_interfaces::srv::Detection::Response> response){
            // creating HSV matrices to store the color filtering
            cv::Mat hsv_frame;
            //std::unique_lock<std::mutex> lock(*frame_mutex);
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
            //lock.unlock();

            // Creating mask matrices which leaves the HSV values (NOT THE DETECTED COLOR)
            cv::Mat goal_mask;
            cv::inRange(hsv_frame, yellow_lower_bound, yellow_upper_bound, goal_mask);

            // Creating matrix for edge detection and
            cv::Mat edges;
            cv::Canny(goal_mask, edges, low_threshold, high_threshold);

            std::vector<cv::Vec4i> linesP;
            cv::HoughLinesP(edges, linesP, rho, theta, threshold, min_line_length, max_line_gap);

            std::vector<cv::Point> midpoints;
            int center_x, center_y, total_lines = 0;

            // checking to see if any lines have been drawn
            if (!linesP.empty()) {
                // going through each drawn line to create midpoints
                for (size_t i = 0; i < linesP.size(); i++) {
                    cv::Vec4i line = linesP[i];
                    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
                    int mid_x = (x1 + x2) / 2;
                    int mid_y = (y1 + y2) / 2;

                    midpoints.push_back(cv::Point(mid_x, mid_y));
                }

                // checking to see if any midpoints have been created
                if (!midpoints.empty()) {
                    int max_x = INT_MIN, min_x = INT_MAX, max_y = INT_MIN, min_y = INT_MAX;

                    for (const auto &point : midpoints) {
                        max_x = std::max(max_x, point.x);
                        min_x = std::min(min_x, point.x);
                        max_y = std::max(max_y, point.y);
                        min_y = std::min(min_y, point.y);
                    }

                    center_x = (min_x + max_x) / 2;
                    center_y = (min_y + max_y) / 2;
                    total_lines++;
                }

                if (total_lines % 5 == 0) {
                    RCLCPP_INFO(this->get_logger(), "CamNode - X: %i Y: %i", center_x, center_y);
                    
                    auto msg = blimp_interfaces::msg::CameraCoord();
                    msg.position = {center_x, center_y};
                    cam_data_publisher_->publish(msg);
                }

                midpoints.clear();

                return;
            }

        }   
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalDetectionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
