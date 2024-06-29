#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/srv/detection.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencv2/opencv.hpp"
#include <vector>
#include <thread>
#include <chrono>

class CamNode : public rclcpp::Node {
public:
    CamNode() : Node("Cam_Node"), frames(0), time_since_start(0.0) {
        // subscribing to the topic "joy"
        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&CamNode::callback_read_joy, this, std::placeholders::_1));

        // setting variable cap_ to default constructor VideoCapture, CAP_V4L2 sets the cap to the proper video channel for linux
        cap_ = cv::VideoCapture(0, cv::CAP_V4L2);

        // setting frame width and height of pi camera
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        control_mode = false; // set for manual mode first
        cam_mode = true;      // set for balloon detection first
        x_button = 0;

        hasDetection = false;
        x_coord = 0;
        y_coord = 0;

        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CamNode::callback_read_image, this));
        RCLCPP_INFO(this->get_logger(), "CamNode - Video Detection has started!");
    }

    void shutdown_node() {
        if (frames > 0 && time_since_start > 0.0) {
            double avg_fps = frames / time_since_start;
            RCLCPP_INFO(this->get_logger(), "Average FPS: %f", avg_fps);
        }
    }

    void shutdown_and_display_fps() {
        shutdown_node(); // Call the private shutdown function
        if (frames > 0 && time_since_start > 0.0) {
            double avg_fps = frames / time_since_start;
            RCLCPP_INFO(this->get_logger(), "Average FPS: %f", avg_fps);
        }
    }

private:
    void callback_read_joy(const sensor_msgs::msg::Joy::SharedPtr button) {
        std::cout << "I have been called!" << std::endl;
        x_button = button->buttons[3];
    }

    void call_balloon_detection_service(std::vector<signed char>& convertedFrame, int rows, int cols) {
        auto client = this->create_client<blimp_interfaces::srv::Detection>("balloon_detection");

        while (!client->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for balloon detection server to be up...");
        }

        auto request = std::make_shared<blimp_interfaces::srv::Detection::Request>();
        request->frame = convertedFrame;
        request->rows = rows;
        request->cols = cols;

        auto future = client->async_send_request(request);

        while (!future.valid()) {}

        /*
        auto response = future.get();

        hasDetection = response->detection;
        x_coord = response->x;
        y_coord = response->y;
        */

        return;
    }

    void call_goal_detection_service(std::vector<signed char>&  convertedFrame, int rows, int cols) {
        auto client = this->create_client<blimp_interfaces::srv::Detection>("goal_detection");
        while (!client->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for balloon detection server to be up...");
        }

        auto request = std::make_shared<blimp_interfaces::srv::Detection::Request>();
        request->frame = convertedFrame;
        request->rows = rows;
        request->cols = cols;

        auto future = client->async_send_request(request);

        while (!future.valid()) {}

        return;
    }

    void callback_read_image() {
        // seeing if button to switch cam mode has been pressed
        if (x_button == 1) {
            cam_mode = !cam_mode;
            sleep(1);
            RCLCPP_INFO(this->get_logger(), "CamNode - Cam Mode has been switched!");
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "CamNode - ERROR: Could not open video source!");
            return;
        }

        auto start = std::chrono::steady_clock::now();

        // checking to see if in manual or autonomous control (true -> manual, false -> autonomous)
        if (!control_mode) {
            // capture frame and setting it to matrix frame
            cv::Mat frame;
            cap_ >> frame;

            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "CamNode - ERROR: frame is empty!");
                return;
            }

            // converting frame into std::vector<unsigned char> to be sent through service
            std::vector<signed char> frameVector;
            if (frame.isContinuous()) {
                frameVector.assign(frame.datastart, frame.dataend);
            } else {
                for (int i = 0; i < frame.rows; ++i) {
                    frameVector.insert(frameVector.end(), frame.ptr<signed char>(i), frame.ptr<signed char>(i) + frame.cols * frame.channels());
                }
            }

            // checking to see if balloon or goal detection (true -> balloon, false -> goal)
            if (cam_mode) {
                // balloon detection
                //call_thread = std::thread(&CamNode::call_balloon_detection_service, this, std::ref(frameVector), frame.rows, frame.cols);
                call_balloon_detection_service(frameVector, frame.rows, frame.cols);
            } else {
                // goal detection
                //call_thread = std::thread(&CamNode::call_goal_detection_service, this, std::ref(frameVector), frame.rows, frame.cols);
                call_goal_detection_service(frameVector, frame.rows, frame.cols);
            }

            frames += 1;
            auto end = std::chrono::steady_clock::now();
            auto elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            time_since_start += elapsed_seconds.count();
            RCLCPP_INFO(this->get_logger(), "Time per frame: %f", elapsed_seconds.count());
        }

        /*
            TODO:
              - fix stupid ass lines above
              - create an the 10 average for detected coords
              - possibly create something that will clear when recently switched from balloon to goal or vice versa
        */
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread call_thread;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    bool control_mode;
    bool cam_mode;
    int x_button;
    int hasDetection;
    int x_coord;
    int y_coord;
    int frames;
    double time_since_start;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamNode>();
    rclcpp::spin(node);
    node->shutdown_and_display_fps(); // Call the method to display FPS
    rclcpp::shutdown();
    return 0;
}

