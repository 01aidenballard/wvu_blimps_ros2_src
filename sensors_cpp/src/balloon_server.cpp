#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/srv/detection.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class BalloonDetectionServerNode : public rclcpp::Node {
    public:
        BalloonDetectionServerNode() : Node("balloon_detection_server") {
            server_ = this.create_service<blimp_interfaces::srv::detection>(
                "balloon_detection", 
                std::bind(&BalloonDetectionServerNode::callback_balloon_detect, 
                this, _1, _2, _3));
            RCLCPP_INFO(this->get_logger(), "Balloon Detection Server has been started!");
        }
    
    private:
        void callback_balloon_detect(const blimp_interfaces::srv::detection::Request::SharedPrt request,
            const blimp_interfaces::srv::detection::Response::SharedPrt response) {
            // Converting vector back into cv::Mat (98% sure will have to change)
            cv::Mat frame(request->rows, request->cols, uchar, (void*)request->frame.data());

            // Creating HSV matrices to store the color filtering
            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

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

            int balloonX = 0;
            int baloonY = 0;

            if (largest_contour.size.width != 0 && largest_contour.size.height != 0) {
                cv::Point2f center = largest_contour.center;
                int radius = std::max(largest_contour.size.width, largest_contour.size.height) / 2;
                if ((radius >= minimum_radius && radius <= maximum_radius) && (center.x >= 0 && center.x < frame.cols && center.y >= 0 && center.y < frame.rows)) {
                    hasDetection = true;
                    balloonX = center.x;
                    balloonY = center.y;
                } else {
                    hasDetection = false;
                }
            } else {
                hasDetection = false;
                returnX = 0;
            }

            response->detection = hasDetection;
            reponse->x = balloonX;
            reponse->y = balloonY;
        }

        bool hasDetection = false;
        const int minimum_radius = 15;
        const int maximum_radius = 300;

        cv::Scalar purple_lower_bound = cv::Scalar(120, 40, 30);
        cv::Scalar purple_upper_bound = cv::Scalar(150, 255, 255);
        cv::Scalar green_lower_bound = cv::Scalar(41, 80, 80);
        cv::Scalar green_upper_bound = cv::Scalar(56, 255, 255);

        rclcpp::Service<blimp_interfaces::srv::detection>::SharedPtr server_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalloonDetectionServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
