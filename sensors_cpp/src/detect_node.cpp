#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "opencv2/opencv.hpp"
#include "vector"



class CamNode : public rclcpp::Node
{
public:
    CamNode() : Node("cam_node")
    {
        cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);
        cap_ = cv::VideoCapture(0);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        frame_count_ = 0;
        minimum_radius_ = 20;
        findGoal = true;

        rho = 1;
        theta = CV_PI / 180;
        threshold = 75;
        min_line_length = 50;
        max_line_gap = 30;

        total_lines = 0;
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CamNode::callback_read_image, this));
        RCLCPP_INFO(this->get_logger(), "Video Detection has Started, press w to switch detection");
    }

public:
    
    int counter_timer = 0;
    void callback_read_image()
    {
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not open video source.");
            return;
        }

        counter_timer++;

        if (counter_timer == 25){
                counter_timer = 0;
               findGoal = !findGoal;
                if(findGoal){
                        RCLCPP_INFO(this->get_logger(), "Goal Detection has started!");
                } else {
                        RCLCPP_INFO(this->get_logger(), "Balloon Detection has stated!");
                }
        }

        cv::Mat frame, goal;
        cap_ >> frame;
        cap_ >> goal;

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not read frame.");
            return;
        }

        cv::Mat hsv_frame, goal_hsv;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::cvtColor(goal, goal_hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask_1, mask_2;
        cv::Mat mask_goal;

        //Yellow
        cv::Scalar goal_lower_bound = cv::Scalar(28, 80, 120);
        cv::Scalar goal_upper_bound = cv::Scalar(36, 255, 255);

        //Orange
        //cv::Scalar goal_lower_bound = cv::Scalar(1, 120, 50);
        //cv::Scalar goal_lower_bound = cv::Scalar(12, 255, 255);
        
        cv::Scalar lower_bound_1 = cv::Scalar(39, 80, 80);
        cv::Scalar upper_bound_1 = cv::Scalar(75, 255, 255);

        cv::Scalar lower_bound_2 = cv::Scalar(128, 80, 80);
        cv::Scalar upper_bound_2 = cv::Scalar(160, 170, 255);

        cv::inRange(hsv_frame, goal_lower_bound, goal_upper_bound, mask_goal);

        cv::inRange(hsv_frame, lower_bound_1, upper_bound_1, mask_1);
        cv::inRange(hsv_frame, lower_bound_2, upper_bound_2, mask_2);

        if (!(findGoal)) 
        {

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
                    if (radius >= minimum_radius && radius <= maximum_radius) {
                        if (center.x >= 0 && center.x < frame.cols && center.y >= 0 && center.y < frame.rows) {
                            detected_coords.push_back(center);
                            // Draw a circle around the detected coordinate
                           // cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
                            // Print the detected coordinates
                          // std::cout << "Detected X: " << center.x << ", Detected Y: " << center.y << std::endl;
                        }
                    }
                }

                    // Calculate and print average coordinates
                    total_x = 0;
                    total_y = 0;
                    for (const auto &coord : detected_coords) {
                       total_x += coord.x;
                        total_y += coord.y;
                    }
                    if (!detected_coords.empty()) {
                       // RCLCPP_INFO(this->get_logger(),  "Average X: " << total_x / detected_coords.size() << ", Average Y: " << total_y / detected_coords.size());
                        auto msg = blimp_interfaces::msg::CameraCoord();
                        avg_x = std::round(total_x/detected_coords.size());
                        avg_y = std::round(total_y/detected_coords.size());
                        msg.position = {avg_x,avg_y};
                        cam_data_publisher_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "coords} avg_x: %i, avg_y: %i", avg_x, avg_y);
                    }
                    // Clear the detected coordinates for the next 10 frames
                    detected_coords.clear();

                //cv::imshow("Detected Color", frame);

        }

        else
        { 

          cv::Mat edges;
          cv::Canny(mask_goal, edges, low_threshold, high_threshold);

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
                 RCLCPP_INFO(this->get_logger(), "DEsolation."); 
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
  
                 // cv::circle(frame, cv::Point(max_x, max_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(max_x, min_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(min_x, max_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(min_x, min_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(center_x, center_y), 5, cv::Scalar(255, 0 , 0), -1);
  
  
              } 
                  auto msg = blimp_interfaces::msg::CameraCoord();
                  msg.position = {center_x, center_y};
                  cam_data_publisher_->publish(msg);
                  RCLCPP_INFO(this->get_logger(), "X: %i Y: %i", center_x, center_y);

              midpoints.clear();
              //std::cout << "Goal - X: " << center_x << ", Y: " << center_y << std::endl;
          }
       }

    
        // cv::imshow("Detection", frame);

        // if(cv::waitKey(1) == 27){
          // return;
        // }

    
    }



    const int minimum_radius = 20;
    const int maximum_radius = 300;
    int avg_x;
    int avg_y;
    int frame_count = 0;
    std::vector<cv::Point2f> detected_coords;
    float total_x = 0;
    float total_y = 0;

    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t frame_count_;
    int minimum_radius_;
    int center_x;
    int center_y;
    float rho;
    float theta;
    int threshold;
    int min_line_length;
    int max_line_gap;
    int low_threshold;
    int high_threshold;
    int total_lines;
    bool findGoal;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    node->cap_.release();
    cv::destroyAllWindows();
    return 0;
}
