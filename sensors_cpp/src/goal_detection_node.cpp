#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <blimp_interfaces/msg/camera_coord.hpp>

class GoalDetectNode : public rclcpp::Node {
public:
    GoalDetectNode() : Node("goal_detect_cpp") {
        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        if (!cap.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "Error: Could not open video source.");
            return;
        }

        rho = 1;
        theta = CV_PI / 180;
        threshold = 75;
        min_line_length = 50;
        max_line_gap = 30;

        low_threshold = 250;
        high_threshold = 300;

        total_x = 0;
        total_y = 0;
        total_lines = 0;

        cam_data_publisher = this->create_publisher<blimp_interfaces::msg::CameraCoord>("goal_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&GoalDetectNode::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Goal Detection CPP has Started!");
    }

public:
    void timerCallback() {
        cv::Mat frame;
        cap.read(frame);

        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Error: Could not read frame.");
        }

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

                //cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
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

                //cv::circle(frame, cv::Point(max_x, max_y), 5, cv::Scalar(0, 0, 255), -1);
                //cv::circle(frame, cv::Point(max_x, min_y), 5, cv::Scalar(0, 0, 255), -1);
                //cv::circle(frame, cv::Point(min_x, max_y), 5, cv::Scalar(0, 0, 255), -1);
                //cv::circle(frame, cv::Point(min_x, min_y), 5, cv::Scalar(0, 0, 255), -1);
                //cv::circle(frame, cv::Point(center_x, center_y), 5, cv::Scalar(255, 0 , 0), -1);


            }

            if (total_lines % 5 == 0) {

                auto msg = blimp_interfaces::msg::CameraCoord();
                msg.position = {center_x, center_y};
                cam_data_publisher->publish(msg);

            }
            midpoints.clear();
            //std::cout << "Goal - X: " << center_x << ", Y: " << center_y << std::endl;
            //RCLCPP_INFO(this->get_logger(), "X: %d Y: %d", center_x, center_y);
        }


        //cv::imshow("Detection", frame);

        //if (cv::waitKey(1)) {
            //return;
        //}
    }

    cv::VideoCapture cap;
    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher;
    rclcpp::TimerBase::SharedPtr timer_;

    int center_x;
    int center_y;
    float rho;
    float theta;
    int threshold;
    int min_line_length;
    int max_line_gap;
    int low_threshold;
    int high_threshold;
    int total_x;
    int total_y;
    int total_lines;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    node->cap.release();
    cv::destroyAllWindows();
    return 0;
}
