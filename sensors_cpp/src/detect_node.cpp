#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <blimp_interfaces/msg/camera_coord.hpp>

class DetectionNode : public rclcpp::Node {
public:
    DetectionNode() : Node("detection_node_cpp"), method_flag(0) {
        // Open the camera once
        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        if (!cap.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "Error: Could not open video source.");
            return;
        }

        // Initialize publishers and timer
        goal_publisher = this->create_publisher<blimp_interfaces::msg::CameraCoord>("goal_data", 10);
        balloon_publisher = this->create_publisher<blimp_interfaces::msg::CameraCoord>("balloon_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&DetectionNode::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Detection CPP Node has Started!");
    }

    void timerCallback() {
        cv::Mat frame;
        cap.read(frame);

        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Error: Could not read frame.");
            return;
        }

        if (method_flag == 0) {
            publishGoalCoords(frame);
        } else {
            publishBalloonCoords(frame);
        }
    }

    void toggleMethod() {
        method_flag = (method_flag + 1) % 2;
        RCLCPP_INFO(this->get_logger(), "Switched method to: %s", method_flag == 0 ? "Goal Detection" : "Balloon Detection");
    }

private:
    cv::VideoCapture cap;
    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr goal_publisher;
    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr balloon_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    int method_flag;
    int rho = 1;
    double theta = CV_PI / 180;
    int threshold = 75;
    int min_line_length = 50;
    int max_line_gap = 30;
    int low_threshold = 50;
    int high_threshold = 150;
    int total_lines = 0;
    int center_x = 0;
    int center_y = 0;

    void publishGoalCoords(const cv::Mat& frame) {
        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        cv::Scalar lower_bound = cv::Scalar(23, 80, 80);
        cv::Scalar upper_bound = cv::Scalar(45, 255, 255);

        cv::Mat mask_1;
        cv::inRange(hsv_frame, lower_bound, upper_bound, mask_1);

        cv::Mat edges;
        cv::Canny(mask_1, edges, low_threshold, high_threshold);

        std::vector<cv::Vec4i> linesP;
        cv::HoughLinesP(edges, linesP, rho, theta, threshold, min_line_length, max_line_gap);

        std::vector<cv::Point> midpoints;

        if (!linesP.empty()) {
            for (size_t i = 0; i < linesP.size(); i++) {
                cv::Vec4i line = linesP[i];
                int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
                int mid_x = (x1 + x2) / 2;
                int mid_y = (y1 + y2) / 2;

                midpoints.push_back(cv::Point(mid_x, mid_y));
            }

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
                auto msg = blimp_interfaces::msg::CameraCoord();
                msg.position = {center_x, center_y};
                goal_publisher->publish(msg);
            }
            midpoints.clear();
        }
    }

    void publishBalloonCoords(const cv::Mat& frame) {
        const int minimum_radius = 20;
        int avg_x;
        int avg_y;
        int frame_count = 0;
        std::vector<cv::Point2f> detected_coords;
        float total_x = 0;
        float total_y = 0;

        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        cv::Scalar lower_bound_1 = cv::Scalar(39, 80, 80);
        cv::Scalar upper_bound_1 = cv::Scalar(75, 255, 255);

        cv::Scalar lower_bound_2 = cv::Scalar(115, 80, 80);
        cv::Scalar upper_bound_2 = cv::Scalar(160, 255, 255);

        cv::Mat mask_1, mask_2;
        cv::inRange(hsv_frame, lower_bound_1, upper_bound_1, mask_1);
        cv::inRange(hsv_frame, lower_bound_2, upper_bound_2, mask_2);

        std::vector<std::vector<cv::Point>> contours_1, contours_2, all_contours;
        cv::findContours(mask_1, contours_1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_2, contours_2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        all_contours.insert(all_contours.end(), contours_1.begin(), contours_1.end());
        all_contours.insert(all_contours.end(), contours_2.begin(), contours_2.end());

        cv::RotatedRect largest_contour;
        double largest_contour_area = 0;

        for (const auto &contour : all_contours) {
            double contour_area = cv::contourArea(contour);
            if (contour_area > largest_contour_area) {
                largest_contour = cv::minAreaRect(contour);
                largest_contour_area = contour_area;
            }
        }

        if (largest_contour.size.width != 0 && largest_contour.size.height != 0) {
            cv::Point2f center = largest_contour.center;
            int radius = std::max(largest_contour.size.width, largest_contour.size.height) / 2;
            if (radius >= minimum_radius) {
                if (center.x >= 0 && center.x < frame.cols && center.y >= 0 && center.y < frame.rows) {
                    detected_coords.push_back(center);
                    cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
                }
            }
        }

        frame_count++;

        if (frame_count % 10 == 0) {
            total_x = 0;
            total_y = 0;
            for (const auto &coord : detected_coords) {
                total_x += coord.x;
                total_y += coord.y;
            }
            if (!detected_coords.empty()) {
                auto msg = blimp_interfaces::msg::CameraCoord();
                avg_x = std::round(total_x/detected_coords.size());
                avg_y = std::round(total_y/detected_coords.size());
                msg.position = {avg_x,avg_y};
                balloon_publisher->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published goal coordinates: x=%d, y=%d", center_x, center_y);
            }
            detected_coords.clear();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto detection_node = std::make_shared<DetectionNode>();

    // Toggle between detection methods on key press
    std::cout << "Press 'g' to switch to Goal Detection, 'b' to switch to Balloon Detection, and 'q' to quit." << std::endl;
    char key;
    while (true) {
        std::cin >> key;
        if (key == 'q') {
            break;
        } else if (key == 'g') {
            detection_node->toggleMethod();
        } else if (key == 'b') {
            detection_node->toggleMethod();
        }
    }

    rclcpp::shutdown();
    return 0;
}

