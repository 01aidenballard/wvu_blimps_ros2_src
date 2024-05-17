#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/srv/detection.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

// for more parameters add more place holders
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

/**
 * TODO:
 *  - figure out instantiated variables
*/

class GoalDetectionServer : public rclcpp::Node {
    public:
        GoalDetectionServer() : Node("Goal_detection_server") {
            server_ = this.create_service<blimp_interfaces::srv:detection>(
                "detection",
                std::bind(&GoalDetectionServer::callback_goal_detect,
                this, _1, _2, _3));

            // setting variables for start
            rho = 1;
            theta = CV_PI / 180;
            threshold = 75;
            min_line_length = 50;
            max_line_gap = 30;

            RCLCPP_INFO(this->get_logger(), "Goal Detection has started.");
        }
    private:

        void callback_goal_detect(const blimp_interfaces::srv::detection::Request::SharedPtr request,
        const blimp_interfaces::srv::detection::Responce::SharedPtr responce){
            // converting vector back into cv::Mat (98% sure will have to change)
            cv::Mat frame(request->rows, request->cols, uchar, (void*)request->frame.data());

            // creating hsv matricies to store the color filtering for the goal detection
            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            cv::Mat mask_goal;

            cv::inRange(hsv_frame, goal_lower_bound, goal_upper_bound, mask_goal);
            // matrix for edge detection
            cv::Mat edges;

            cv::Canny(mask_goal, edges, low_threshold, high_threshold);

            std::vector<cv::Vec4i> linesP;

            cv::HoughLinesP(edges, linesP, rho, theta, threshold, min_line_length, max_line_gap);
            std::vector<cv::Point> midpoints;
            if (!linesP.empty()) {
                for (size_t i = 0; i< linesP.size(); i++) {
                    cv::Vec4i line = linesP[i];
                    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

                    int mid_x = (x1 + x2)/2;
                    int mid_y = (y1 + y2)/2;

                    midpoints.push_back(cv::Point(mid_x, mid_y));

                }

                if (!midpoints.empty()) {
                    int max_x = NT_MIN, min_x = INT_MAX, max_y = INT_MIN, min_y = INT_MAX;

                        for (const auto &point : midpoints) {
                            max_x = std::max(max_x, point.x);
                            min_x = std::min(min_x, point.x);
                            max_y = std::max(max_y, point.y);
                            min_y = std::min(min_y, point.y);
                        }
  		            //finding the midpoint of the midpoints in order to find the center
                    center_x = (min_x + max_x) / 2;
                    center_y = (min_y + max_y) / 2;
                    total_lines++;
                }
                responce->x = center_x;
                responce->y = center_y;
                midpoints.clear();
            }
        }

        //yellow
        cv::Scalar goal_lower_bound = cv::Scalar(28,80,120);
        cv::Scalar goal_upper_bound = cv::Scalar(36, 255, 255);

        float rho;
        float theta;
        int threshold;
        int min_line_length;
        int max_line_gap;
        int low_threshold;
        int high_threshold;
        int total_lines;
        //orange
	    //cv::Scalar goal_lower_bound = cv::Scalar(1,120,50);
	    //cv::Scalar goal_upper_bound = cv::Scalar(12,255,255);

        rclcpp::Service<blimp_interfaces::srv::detection>::SharedPointer server_;
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalDetectionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
