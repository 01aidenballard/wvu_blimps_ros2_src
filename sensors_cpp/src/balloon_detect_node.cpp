#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "vector"
#include "opencv2/opencv.hpp"

class BalloonDetectNode: public rclcpp::Node {
    
public:

    BalloonDetectNode(): Node("Ball_detect_cpp") {

        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // x-axis
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);  // y-axis

        if (!cap.isOpened()) {
            //std::cout << "Error: Could not open video source." << std::endl;
            RCLCPP_INFO(this->get_logger(), "Error: Could not open video source.");
            return;
        }

        cam_data_publisher = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30),std::bind(&BalloonDetectNode::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Balloon Detection CPP has Started!");
    }

private:

    void timerCallback() {
        cv::Mat frame;
        cap.read(frame);

        if (frame.empty()) {
            //std::cout << "Error: Could not read frame." << std::endl;
            RCLCPP_INFO(this->get_logger(), "Error: Could not read frame.");
        }

        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        //cv::Scalar lower_bound_1 = cv::Scalar(39, 80, 80);
        //cv::Scalar upper_bound_1 = cv::Scalar(75, 255, 255);

        cv::Scalar lower_bound_2 = cv::Scalar(/*115*/128.5, 80, 80);
        cv::Scalar upper_bound_2 = cv::Scalar(160, 170, 255);

        cv::Mat/* mask_1,*/ mask_2;
       // cv::inRange(hsv_frame, lower_bound_1, upper_bound_1, mask_1);
        cv::inRange(hsv_frame, lower_bound_2, upper_bound_2, mask_2);

        std::vector<std::vector<cv::Point>>/* contours_1,*/ contours_2, all_contours;
       // cv::findContours(mask_1, contours_1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_2, contours_2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
       // all_contours.insert(all_contours.end(), contours_1.begin(), contours_1.end());
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
                    // Draw a circle around the detected coordinate
                    cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
                    // Print the detected coordinates
                   // std::cout << "Detected X: " << center.x << ", Detected Y: " << center.y << std::endl;
                }
            }
        }

        frame_count++;

        if (frame_count % 10 == 0) {
            // Calculate and print average coordinates
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
		RCLCPP_INFO(this->get_logger(), "X: %d Y: %d", avg_x, avg_y);
                msg.position = {avg_x,avg_y};
                cam_data_publisher->publish(msg);
            }
            // Clear the detected coordinates for the next 10 frames
            detected_coords.clear();
        }

        //cv::imshow("Detected Color", frame);
    }


    const int minimum_radius = 20;
    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoCapture cap;
    int avg_x;
    int avg_y;
    int frame_count = 0;
    std::vector<cv::Point2f> detected_coords;
    float total_x = 0;
    float total_y = 0;

};

int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BalloonDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
