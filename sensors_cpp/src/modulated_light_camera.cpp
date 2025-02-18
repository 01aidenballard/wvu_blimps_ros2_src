// #include "rclcpp/rclcpp.hpp"
// #include "blimp_interfaces/msg/camera_coord.hpp"
// #include "blimp_interfaces/msg/bool.hpp"
// #include "sensor_msgs/msg/joy.hpp"
// #include "opencv2/opencv.hpp"
// #include <thread>
// #include <mutex>
// #include <deque>
// #include <httplib.h>
// #include <cmath>

// class CamNode : public rclcpp::Node {
// public:
//     CamNode() : Node("cam_node"), current_frame_(), frame_mutex_(), intensity_buffer_(), max_buffer_size_(100) {
//         // ROS2 Publishers
//         cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);
//         cam_flag_publisher_ = this->create_publisher<blimp_interfaces::msg::Bool>("cam_flag", 3);

//         subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
//             "joy", 10, std::bind(&CamNode::callback_read_joy, this, std::placeholders::_1));

//         subscriber_net_servo = this->create_subscription<blimp_interfaces::msg::Bool>(
//             "net_flag", 10, std::bind(&CamNode::callback_read_net, this, std::placeholders::_1));

//         // Initialize OpenCV video capture
//         cap_ = cv::VideoCapture(0, cv::CAP_V4L2);
//         cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//         cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

//         if (!cap_.isOpened()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
//             rclcpp::shutdown();
//         }

//         // Start Flask server thread
//         server_thread_ = std::thread(&CamNode::startFlaskServer, this);

//         // Start timer for periodic image capture
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(30), std::bind(&CamNode::callback_read_image, this));

//         RCLCPP_INFO(this->get_logger(), "Video Detection has Started.");
//     }

//     ~CamNode() {
//         cap_.release();
//         if (server_thread_.joinable()) {
//             server_thread_.join();
//         }
//     }

//     void callback_read_joy(const sensor_msgs::msg::Joy::SharedPtr button) {
//         x_button = button->buttons[3];
//     }

//     void callback_read_net(const blimp_interfaces::msg::Bool &msg) {
//         net_flag = msg.flag;
//         RCLCPP_INFO(this->get_logger(), "Received net_flag: %s", net_flag ? "true" : "false");
//     }

//     void callback_read_image() {
//         cv::Mat frame;
//         cap_ >> frame;
//         if (frame.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "Error: Could not read frame.");
//             return;
//         }

//         // HSV processing and masking
//         cv::Mat hsv_frame, mask_goal;
//         cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
//         cv::Scalar lower_red(0, 81, 201);
//         cv::Scalar upper_red(179, 200, 255);
//         cv::inRange(hsv_frame, lower_red, upper_red, mask_goal);

//         // Update intensity buffer
//         double avg_intensity = cv::mean(mask_goal)[0] / 255.0;
//         intensity_buffer_.push_back(avg_intensity);

//         if (intensity_buffer_.size() > max_buffer_size_) {
//             intensity_buffer_.pop_front();
//         }

//         // Detect frequency using peak detection
//         bool signal_detected = false;
//         if (intensity_buffer_.size() == max_buffer_size_) {
//             double frequency = compute_frequency_from_peaks(intensity_buffer_, 30.0); // 30ms frame time
//             RCLCPP_INFO(this->get_logger(), "Detected frequency: %.2f Hz", frequency);

//             if (frequency >= 5.0 && frequency <= 8.0) { // Adjusted range for valid signal
//                 signal_detected = true;
//                 RCLCPP_INFO(this->get_logger(), "Modulated light detected in range 5-8 Hz.");
//             }
//         }

//         if (signal_detected) {
//             // Process and publish masked frame with coordinates
//             process_frame_and_publish(mask_goal, frame);
//         } else {
//             // Publish unmasked frame if no signal detected
//             publish_camera_feed(frame);
//         }
//     }

// private:
//     cv::VideoCapture cap_;
//     cv::Mat current_frame_;
//     std::mutex frame_mutex_;
//     std::thread server_thread_;

//     int x_button = 0;
//     bool net_flag = false;

//     std::deque<double> intensity_buffer_;
//     const size_t max_buffer_size_;

//     rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;
//     rclcpp::Publisher<blimp_interfaces::msg::Bool>::SharedPtr cam_flag_publisher_;
//     rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
//     rclcpp::Subscription<blimp_interfaces::msg::Bool>::SharedPtr subscriber_net_servo;
//     rclcpp::TimerBase::SharedPtr timer_;

//     void publish_camera_feed(const cv::Mat& frame) {
//         {
//             std::lock_guard<std::mutex> lock(frame_mutex_);
//             current_frame_ = frame.clone();
//         }
//         RCLCPP_INFO(this->get_logger(), "Published normal camera feed.");
//     }

//     void process_frame_and_publish(const cv::Mat& mask_goal, cv::Mat& frame) {
//         std::vector<std::vector<cv::Point>> contours;
//         cv::findContours(mask_goal, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         cv::RotatedRect largest_contour;
//         double largest_contour_area = 0;
//         std::vector<cv::Point2f> detected_coords;

//         for (const auto& contour : contours) {
//             double contour_area = cv::contourArea(contour);
//             if (contour_area > largest_contour_area) {
//                 largest_contour = cv::minAreaRect(contour);
//                 largest_contour_area = contour_area;
//             }
//         }

//         if (largest_contour.size.width != 0 && largest_contour.size.height != 0) {
//             cv::Point2f center = largest_contour.center;
//             int radius = std::max(largest_contour.size.width, largest_contour.size.height) / 2;

//             if (radius >= 2 && radius <= 100) {
//                 detected_coords.push_back(center);
//                 cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
//             }
//         }

//         if (!detected_coords.empty()) {
//             float total_x = 0, total_y = 0;
//             for (const auto& coord : detected_coords) {
//                 total_x += coord.x;
//                 total_y += coord.y;
//             }
//             int avg_x = static_cast<int>(total_x / detected_coords.size());
//             int avg_y = static_cast<int>(total_y / detected_coords.size());

//             auto msg = blimp_interfaces::msg::CameraCoord();
//             msg.position = {avg_x, avg_y};
//             cam_data_publisher_->publish(msg);

//             auto flag_msg = blimp_interfaces::msg::Bool();
//             flag_msg.flag = true;
//             cam_flag_publisher_->publish(flag_msg);

//             RCLCPP_INFO(this->get_logger(), "Published detection data: X = %d, Y = %d", avg_x, avg_y);

//             cv::Mat masked_frame_3channel;
//             cv::cvtColor(mask_goal, masked_frame_3channel, cv::COLOR_GRAY2BGR);

//             cv::Mat combined_frame;
//             cv::hconcat(frame, masked_frame_3channel, combined_frame);

//             {
//                 std::lock_guard<std::mutex> lock(frame_mutex_);
//                 current_frame_ = combined_frame.clone();
//             }
//         }
//     }

//     void startFlaskServer() {
//         httplib::Server server;

//         server.Get("/video_feed", [this](const httplib::Request&, httplib::Response& res) {
//             res.set_content_provider(
//                 "multipart/x-mixed-replace; boundary=frame",
//                 [this](size_t /*offset*/, httplib::DataSink& sink) -> bool {
//                     while (true) {
//                         std::vector<uchar> buffer;
//                         {
//                             std::lock_guard<std::mutex> lock(frame_mutex_);
//                             if (!current_frame_.empty()) {
//                                 cv::imencode(".jpg", current_frame_, buffer);
//                             } else {
//                                 continue;
//                             }
//                         }

//                         std::string frame_boundary = "--frame\r\n"
//                                                      "Content-Type: image/jpeg\r\n\r\n";
//                         sink.write(frame_boundary.c_str(), frame_boundary.size());
//                         sink.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
//                         sink.write("\r\n", 2);

//                         std::this_thread::sleep_for(std::chrono::milliseconds(33));
//                     }
//                     return true;
//                 });
//         });

//         server.listen("0.0.0.0", 5000);
//     }

//     double compute_frequency_from_peaks(const std::deque<double>& signal, double sampling_rate) {
//         std::vector<size_t> peaks;

//         for (size_t i = 1; i < signal.size() - 1; ++i) {
//             if (signal[i] > signal[i - 1] && signal[i] > signal[i + 1]) {
//                 peaks.push_back(i);
//             }
//         }

//         if (peaks.size() < 2) {
//             return 0.0;
//         }

//         std::vector<double> intervals;
//         for (size_t i = 1; i < peaks.size(); ++i) {
//             intervals.push_back(static_cast<double>(peaks[i] - peaks[i - 1]) / sampling_rate);
//         }

//         double avg_interval = std::accumulate(intervals.begin(), intervals.end(), 0.0) / intervals.size();
//         return avg_interval > 0 ? 1.0 / avg_interval : 0.0;
//     }
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CamNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();

//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "blimp_interfaces/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencv2/opencv.hpp"
#include <thread>
#include <mutex>
#include <deque>
#include <httplib.h>
#include <cmath>

class CamNode : public rclcpp::Node {
public:
    CamNode() : Node("cam_node"), current_frame_(), frame_mutex_(), intensity_buffer_(), max_buffer_size_(100) {
        // Declare launch parameters
        camera_width_ = this->declare_parameter<int>("camera_width", 640);
        camera_height_ = this->declare_parameter<int>("camera_height", 480);
        fps_ = this->declare_parameter<int>("fps", 30);
        lower_freq_ = this->declare_parameter<double>("lower_freq", 5.0);
        upper_freq_ = this->declare_parameter<double>("upper_freq", 8.0);

        // ROS2 Publishers
        cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);
        cam_flag_publisher_ = this->create_publisher<blimp_interfaces::msg::Bool>("cam_flag", 3);

        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&CamNode::callback_read_joy, this, std::placeholders::_1));

        subscriber_net_servo = this->create_subscription<blimp_interfaces::msg::Bool>(
            "net_flag", 10, std::bind(&CamNode::callback_read_net, this, std::placeholders::_1));

        // Initialize OpenCV video capture
        cap_ = cv::VideoCapture(0, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            rclcpp::shutdown();
        }

        // Start Flask server thread
        server_thread_ = std::thread(&CamNode::startFlaskServer, this);

        // Start timer for periodic image capture
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps_), std::bind(&CamNode::callback_read_image, this));

        RCLCPP_INFO(this->get_logger(), "Video Detection has Started.");
    }

    ~CamNode() {
        cap_.release();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
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
    
        // HSV processing and masking
        cv::Mat hsv_frame, mask_goal;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::Scalar lower_red(0, 81, 201);
        cv::Scalar upper_red(179, 200, 255);
        cv::inRange(hsv_frame, lower_red, upper_red, mask_goal);

        // Convert to Grayscale
        // cv::Mat gray_frame, mask_goal;
        // cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

        // Adaptive Thresholding to extract bright regions dynamically
        //cv::Mat binary_mask;
        //cv::adaptiveThreshold(gray_frame, mask_goal, 255, 
        //                  cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

    
        // Update intensity buffer
        double avg_intensity = cv::mean(mask_goal)[0] / 255.0;
        intensity_buffer_.push_back(avg_intensity);
    
        // Store timestamps for each frame
        double current_time_ms = this->now().seconds() * 1000.0;  // Convert to milliseconds
        time_stamp_buffer_.push_back(current_time_ms);
    
        // Keep the buffer size limited
        if (intensity_buffer_.size() > max_buffer_size_) {
            intensity_buffer_.pop_front();
            time_stamp_buffer_.pop_front();  // Ensure timestamps match intensity buffer
        }
    
        // Detect frequency using peak detection
        bool signal_detected = false;
        if (intensity_buffer_.size() == max_buffer_size_) {
            double frequency = compute_frequency_from_peaks(intensity_buffer_, time_stamp_buffer_);
            //RCLCPP_INFO(this->get_logger(), "Detected frequency: %.2f Hz", frequency);
    
            if (frequency >= lower_freq_ && frequency <= upper_freq_) {
                signal_detected = true;
                //RCLCPP_INFO(this->get_logger(), "Modulated light detected in range %.2f-%.2f Hz.", lower_freq_, upper_freq_);
            }
        }
    
        // if (signal_detected) {
        //     process_frame_and_publish(mask_goal, frame);
        // } else {
        //     publish_camera_feed(frame);
        // }
        if (signal_detected) {
            process_frame_and_publish(mask_goal, frame);
        } else {
            publish_camera_feed(frame, mask_goal); // Now streams both original & masked
        }
        
    }
    

private:
    cv::VideoCapture cap_;
    cv::Mat current_frame_;
    std::mutex frame_mutex_;
    std::thread server_thread_;

    int x_button = 0;
    bool net_flag = false;
    int camera_width_, camera_height_, fps_;
    double lower_freq_, upper_freq_;

    std::deque<double> intensity_buffer_;
    std::deque<double> time_stamp_buffer_;
    const size_t max_buffer_size_;

    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;
    rclcpp::Publisher<blimp_interfaces::msg::Bool>::SharedPtr cam_flag_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    rclcpp::Subscription<blimp_interfaces::msg::Bool>::SharedPtr subscriber_net_servo;
    rclcpp::TimerBase::SharedPtr timer_;

    // void publish_camera_feed(const cv::Mat& frame) {
    //     {
    //         std::lock_guard<std::mutex> lock(frame_mutex_);
    //         current_frame_ = frame.clone();
    //     }
    //    // RCLCPP_INFO(this->get_logger(), "Published normal camera feed.");
    // }
    void publish_camera_feed(const cv::Mat& frame, const cv::Mat& mask) {
        cv::Mat masked_frame_3channel;
        cv::cvtColor(mask, masked_frame_3channel, cv::COLOR_GRAY2BGR); // Convert to 3-channel for consistency
    
        cv::Mat combined_frame;
        cv::hconcat(frame, masked_frame_3channel, combined_frame); // Show original + filtered side by side
    
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = combined_frame.clone(); // Update the frame to stream via Flask
        }
    
        //RCLCPP_INFO(this->get_logger(), "Published masked camera feed.");
    }
    

    // void process_frame_and_publish(const cv::Mat& mask_goal, cv::Mat& frame) {
    //     std::vector<std::vector<cv::Point>> contours;
    //     cv::findContours(mask_goal, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //     cv::RotatedRect largest_contour;
    //     double largest_contour_area = 0;
    //     std::vector<cv::Point2f> detected_coords;

    //     for (const auto& contour : contours) {
    //         double contour_area = cv::contourArea(contour);
    //         if (contour_area > largest_contour_area) {
    //             largest_contour = cv::minAreaRect(contour);
    //             largest_contour_area = contour_area;
    //         }
    //     }

    //     if (largest_contour.size.width != 0 && largest_contour.size.height != 0) {
    //         cv::Point2f center = largest_contour.center;
    //         int radius = std::max(largest_contour.size.width, largest_contour.size.height) / 2;

    //         if (radius >= 2 && radius <= 100) {
    //             detected_coords.push_back(center);
    //             cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
    //         }
    //     }

    //     if (!detected_coords.empty()) {
    //         float total_x = 0, total_y = 0;
    //         for (const auto& coord : detected_coords) {
    //             total_x += coord.x;
    //             total_y += coord.y;
    //         }
    //         int avg_x = static_cast<int>(total_x / detected_coords.size());
    //         int avg_y = static_cast<int>(total_y / detected_coords.size());

    //         auto msg = blimp_interfaces::msg::CameraCoord();
    //         msg.position = {avg_x, avg_y};
    //         cam_data_publisher_->publish(msg);

    //         auto flag_msg = blimp_interfaces::msg::Bool();
    //         flag_msg.flag = true;
    //         cam_flag_publisher_->publish(flag_msg);

    //         //RCLCPP_INFO(this->get_logger(), "Published detection data: X = %d, Y = %d", avg_x, avg_y);

    //         cv::Mat masked_frame_3channel;
    //         cv::cvtColor(mask_goal, masked_frame_3channel, cv::COLOR_GRAY2BGR);

    //         cv::Mat combined_frame;
    //         cv::hconcat(frame, masked_frame_3channel, combined_frame);

    //         {
    //             std::lock_guard<std::mutex> lock(frame_mutex_);
    //             current_frame_ = combined_frame.clone();
    //         }
    //     }
    // }
    void process_frame_and_publish(const cv::Mat& mask_goal, cv::Mat& frame) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_goal, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
        cv::RotatedRect best_contour;
        double max_score = 0.0;
        std::vector<cv::Point2f> detected_coords;
    
        for (const auto& contour : contours) {
            double contour_area = cv::contourArea(contour);
            //if (contour_area < 0.5) continue;  // Ignore tiny noise contours
    
            cv::RotatedRect rect = cv::minAreaRect(contour);
            cv::Mat mask = cv::Mat::zeros(mask_goal.size(), CV_8U);
            cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, 255, cv::FILLED);
    
            double brightness = cv::mean(mask_goal, mask)[0]; // Get average brightness within the contour
            double aspect_ratio = std::max(rect.size.width, rect.size.height) / std::min(rect.size.width, rect.size.height);
            bool is_circular = (aspect_ratio > 0.8 && aspect_ratio < 1.2); // Considered circular
    
            // Weighted Score: Prioritize brightness and size, with some weight on circularity
            double score = (0.7 * brightness) + (0.3 * contour_area);
            if (is_circular) {
                score *= 1.2; // Give circular objects a slight preference
            }
    
            if (score > max_score) {
                max_score = score;
                best_contour = rect;
            }
        }
    
        if (best_contour.size.width != 0 && best_contour.size.height != 0) {
            cv::Point2f center = best_contour.center;
            int radius = std::max(best_contour.size.width, best_contour.size.height) / 2;
    
            if (radius >= 0 && radius <= 100) {
                detected_coords.push_back(center);
                cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
            }
        }
    
        if (!detected_coords.empty()) {
            float total_x = 0, total_y = 0;
            for (const auto& coord : detected_coords) {
                total_x += coord.x;
                total_y += coord.y;
            }
            int avg_x = static_cast<int>(total_x / detected_coords.size());
            int avg_y = static_cast<int>(total_y / detected_coords.size());
    
            auto msg = blimp_interfaces::msg::CameraCoord();
            msg.position = {avg_x, avg_y};
            cam_data_publisher_->publish(msg);
    
            auto flag_msg = blimp_interfaces::msg::Bool();
            flag_msg.flag = true;
            cam_flag_publisher_->publish(flag_msg);
    
            // RCLCPP_INFO(this->get_logger(), "Published detection data: X = %d, Y = %d", avg_x, avg_y);
    
            cv::Mat masked_frame_3channel;
            cv::cvtColor(mask_goal, masked_frame_3channel, cv::COLOR_GRAY2BGR);
    
            cv::Mat combined_frame;
            cv::hconcat(frame, masked_frame_3channel, combined_frame);
    
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                current_frame_ = combined_frame.clone();
            }
        }
    }
    
    void startFlaskServer() {
        httplib::Server server;

        server.Get("/video_feed", [this](const httplib::Request&, httplib::Response& res) {
            res.set_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [this](size_t /*offset*/, httplib::DataSink& sink) -> bool {
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
                        sink.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
                        sink.write("\r\n", 2);

                        std::this_thread::sleep_for(std::chrono::milliseconds(33));
                    }
                    return true;
                });
        });

        server.listen("0.0.0.0", 5000);
    }

    // double compute_frequency_from_peaks(const std::deque<double>& signal, double sampling_rate) {
    //     std::vector<size_t> peaks;

    //     for (size_t i = 1; i < signal.size() - 1; ++i) {
    //         if (signal[i] > signal[i - 1] && signal[i] > signal[i + 1]) {
    //             peaks.push_back(i);
    //         }
    //     }

    //     if (peaks.size() < 2) {
    //         return 0.0;
    //     }

    //     std::vector<double> intervals;
    //     for (size_t i = 1; i < peaks.size(); ++i) {
    //         intervals.push_back(static_cast<double>(peaks[i] - peaks[i - 1]) / sampling_rate);
    //     }

    //     double avg_interval = std::accumulate(intervals.begin(), intervals.end(), 0.0) / intervals.size();
    //     return avg_interval > 0 ? 1.0 / avg_interval : 0.0;
    // }
    double compute_frequency_from_peaks(const std::deque<double>& signal, const std::deque<double>& timestamps) {
        std::vector<size_t> peaks;
        
        // Find peaks in the intensity signal
        for (size_t i = 1; i < signal.size() - 1; ++i) {
            if (signal[i] > signal[i - 1] && signal[i] > signal[i + 1]) {
                peaks.push_back(i);
            }
        }
    
        if (peaks.size() < 2) {
            return 0.0;
        }
    
        // Compute time intervals between peaks using timestamps
        std::vector<double> time_intervals;
        for (size_t i = 1; i < peaks.size(); ++i) {
            double interval_ms = timestamps[peaks[i]] - timestamps[peaks[i - 1]];  // Time difference in milliseconds
            time_intervals.push_back(interval_ms);
        }
    
        double avg_interval_ms = std::accumulate(time_intervals.begin(), time_intervals.end(), 0.0) / time_intervals.size();
        
        return avg_interval_ms > 0 ? 1000.0 / avg_interval_ms : 0.0;  // Convert ms to Hz
    }
    
    
};
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}