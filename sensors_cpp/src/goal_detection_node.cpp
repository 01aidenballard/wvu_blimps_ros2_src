#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <blimp_interfaces/msg/camera_coord.hpp>


class GoalDetectNode: public rclcpp::Node {


public:

    GoalDetectNode(): Node("goal_detect_cpp") {

        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        if(!cap.isOpened()) {
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
        
        cam_data_publisher = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30),std::bind(&GoalDetectNode::timerCallback, this));
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

        cv::Scalar lower_bound = cv::Scalar(20, 80, 80);
        cv::Scalar upper_bound = cv::Scalar(45, 255, 255);

        cv::Mat mask_1;
        cv::inRange(hsv_frame, lower_bound, upper_bound, mask_1);

        cv::Mat edges;
        cv::Canny(mask_1, edges, low_threshold, high_threshold);

        std::vector<cv::Vec4i> linesP;
        cv::HoughLinesP(edges, linesP, rho, theta, threshold, min_line_length, max_line_gap);

        if (!linesP.empty()) {
            for (size_t i = 0; i < linesP.size(); i++) {
                cv::Vec4i line = linesP[i];
                x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
                mid_x = (x1 + x2) / 2;
                mid_y = (y1 + y2) / 2;

                total_x += mid_x;
                total_y += mid_y;
                total_lines++;

                cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
	
            }
            linesP.clear();
            center_x = total_x / total_lines;
            center_y = total_y / total_lines;
            total_lines = 0;
            total_x = 0;
            total_y = 0;
            cv::circle(frame, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), 5);
            std::cout << "Goal - X: " << center_x << ", Y: " << center_y << std::endl;
            RCLCPP_INFO(this->get_logger(),"X: %d Y: %d", center_x, center_y);
        }

        cv::imshow("Detection", frame);

        if (cv::waitKey(1)) {
            return;
        }


    }

    cv::VideoCapture cap;
    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher;
    rclcpp::TimerBase::SharedPtr timer_;

    int center_x;
    int center_y;
    int frame_count = 0;

    float rho;
    float theta;
    int threshold;
    int min_line_length;
    int max_line_gap;
    int low_threshold;
    int high_threshold;
    int x1;
    int y1;
    int x2;
    int y2;
    int mid_x;
    int mid_y;
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
