#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/esc_input.hpp"
#include <Eigen/Dense>
#include <cmath>

class BalloonEscInput : public rclcpp::Node // MODIFY NAME
{
public:
    BalloonEscInput() : Node("balloon_esc_input") // MODIFY NAME
    {
        Fx1 = 1, Fx2 = 1, Fx3 = 0, Fx4 = 0;
        Fy1 = 0, Fy2 = 0, Fy3 = 0, Fy4 = 0;
        Fz1 = 0, Fz2 = 0, Fz3 = -1, Fz4 = 1;
        lx1 = 0.08, lx2 = 0.08, lx3 = -0.08, lx4 = 0.08;
        ly1 = -0.36, ly2 = 0.36, ly3 = 0, ly4 = 0;
        lz1 = -0.19, lz2 = -0.19, lz3 = 0.30, lz4 = 0.30;

        K = Eigen::DiagonalMatrix<double, 4>(0.000602, 0.000602, 0.00153807, 0.00153807);

        B << -0.632456,
             -0.632456,
             -1.75866439,
             -1.75866439;

        Q <<   Fx1,             Fx2,             Fx3,             Fx4, 
               Fy1,             Fy2,             Fy3,             Fy4, 
               Fz1,             Fz2,             Fz3,             Fz4, 
               Fz1*ly1-Fy1*lz1, Fz2*ly2-Fy2*lz2, Fz3*ly3-Fy3*lz3, Fz4*ly4-Fy4*lz4,
               Fx1*lz1-Fz1*lx1, Fx2*lz2-Fz2*lx2, Fx3*lz3-Fz3*lx3, Fx4*lz4-Fz4*lx4,
               Fy1*lx1-Fx1*ly1, Fy2*lx2-Fx2*ly2, Fy3*lx3-Fx3*ly3, Fy4*lx4-Fx4*ly4;

        T = Q*Q.transpose();
        Qp = Q.transpose()*T.inverse();

        K_inv = K.inverse();

        subscriber_ = this->create_subscription<blimp_interfaces::msg::CartCoord>(
            "forces", 10, std::bind(&BalloonEscInput::callback_force_to_esc, this, std::placeholders::_1));

        publisher_ = this->create_publisher<blimp_interfaces::msg::EscInput>("ESC_balloon_input", 10);
        RCLCPP_INFO(this->get_logger(), "Forces are converted");
    }

private:

    void callback_force_to_esc(const blimp_interfaces::msg::CartCoord::SharedPtr msg) {

        tau << msg->x,msg->y,msg->z,msg->theta,msg->phi,msg->psy;

        F = K_inv*Qp*tau - K_inv*B;
        
        if (F(0) > 1900) {
            F(0) = 1900;
        }
        else if (F(0) < 1050) {
            F(0) = 1050;
        }
        else if (F(1) > 1900) {
            F(1) = 1900;
        }
        else if (F(1) < 1050) {
            F(1) = 1050;
        }
        else if (F(2) > 1900) {
            F(2) = 1900;
        }
        else if (F(2) < 1050) {
            F(2) = 1050;
        }
        else if (F(3) > 1900) {
            F(3) = 1900;
        }
        else if (F(3) < 1050) {
            F(3) = 1050;
        }
        
        auto msg2 = blimp_interfaces::msg::EscInput();
        msg2.esc_pins = {5,6,13,27};
        msg2.esc_pwm = {F(0),F(1),F(2),F(3)};
        publisher_->publish(msg2);

    }


    // Members of the Class

    float Fx1, Fx2, Fx3, Fx4;
    float Fy1, Fy2, Fy3, Fy4;
    float Fz1, Fz2, Fz3, Fz4;
    float lx1, lx2, lx3, lx4;
    float ly1, ly2, ly3, ly4;
    float lz1, lz2, lz3, lz4;
    Eigen::DiagonalMatrix<double, 4> K;
    Eigen::DiagonalMatrix<double, 4> K_inv;
    Eigen::Matrix<double, 6,1> tau;
    Eigen::Matrix<double, 6,4> Q;
    Eigen::Matrix<double, 6,6> T;
    Eigen::Matrix<double, 4,6> Qp;
    Eigen::Matrix<double, 4,1> F;
    Eigen::Matrix<double, 4,1> B;

    rclcpp::Subscription<blimp_interfaces::msg::CartCoord>::SharedPtr subscriber_;
    rclcpp::Publisher<blimp_interfaces::msg::EscInput>::SharedPtr publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalloonEscInput>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}