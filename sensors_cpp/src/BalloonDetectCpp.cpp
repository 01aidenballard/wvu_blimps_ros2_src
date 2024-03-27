#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "opencv2/opencv.hpp"

class BallonDetectNode : public rclcpp::Node
{
public:
	BalloonDetectNode() : Node("balloon_detect")
	{
	}

private:
};

int main(int argc, char **argv)
{
	rclcpp::int(argc,argv);
	auto node = std::make_shared<BalloonDetectNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
