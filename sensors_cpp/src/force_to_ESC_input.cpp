#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/esc_input.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <sstream>

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

        T = Q.transpose()*Q;
        Qp = T.inverse()*Q.transpose();

        K_inv = K.inverse();

        // // Assuming you're inside a class method, and you have an `Eigen::IOFormat` declared
        // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

        // // Get the logger
        // auto logger = this->get_logger();

        // // Stringstream for converting matrices to strings
        // std::stringstream ss;

        // // Printing K (DiagonalMatrix)
        // ss << K.toDenseMatrix().format(CleanFmt);
        // RCLCPP_INFO(logger, "Matrix K:\n%s", ss.str().c_str());
        // ss.str(""); // Clearing the stringstream

        // // Printing B (Matrix)
        // ss << B.format(CleanFmt);
        // RCLCPP_INFO(logger, "Matrix B:\n%s", ss.str().c_str());
        // ss.str("");

        // // Printing Q (Matrix)c

        // ss << Q.format(CleanFmt);
        // RCLCPP_INFO(logger, "Matrix Q:\n%s", ss.str().c_str());
        // ss.str("");

        // // Printing T (Matrix)
        // ss << T.format(CleanFmt);
        // RCLCPP_INFO(logger, "Matrix T:\n%s", ss.str().c_str());
        // ss.str("");

        // // Printing Qp (Matrix)
        // ss << Qp.format(CleanFmt);
        // RCLCPP_INFO(logger, "Matrix Qp:\n%s", ss.str().c_str());
        // ss.str("");

        // // Printing K_inv (DiagonalMatrix)
        // ss << K_inv.toDenseMatrix().format(CleanFmt);
        // RCLCPP_INFO(logger, "Matrix K_inv:\n%s", ss.str().c_str());
        // ss.str("");

        subscriber_ = this->create_subscription<blimp_interfaces::msg::CartCoord>(
            "forces", 10, std::bind(&BalloonEscInput::callback_force_to_esc, this, std::placeholders::_1));

        publisher_ = this->create_publisher<blimp_interfaces::msg::EscInput>("ESC_balloon_input", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&BalloonEscInput::callback_timer, this));
        RCLCPP_INFO(this->get_logger(), "Forces are converted");
        tau.setZero();
    }

private:
    void callback_force_to_esc(const blimp_interfaces::msg::CartCoord::SharedPtr msg) {
        //msg->x
        tau << msg->x+0.15,
                  msg->y,
                  msg->z,
                  msg->theta,
                  msg->phi,
                  msg->psy;
    }
    void callback_timer() {

        Eigen::Matrix<double, 4,1> F = K_inv*Qp*tau - K_inv*B;
        
        if (F(0,0) > 1900) {
            F(0,0) = 1900;
        }
        else if (F(0,0) < 1050) {
            F(0,0) = 1050;
        }

        if (F(1,0) > 1900) {
            F(1,0) = 1900;
        }
        else if (F(1,0) < 1050) {
            F(1,0) = 1050;
        }

        if (F(2,0) > 1900) {
            F(2,0) = 1900;
        }
        else if (F(2,0) < 1050) {
            F(2,0) = 1050;
        }

        if (F(3,0) > 1900) {
            F(3,0) = 1900;
        }
        else if (F(3,0) < 1050) {
            F(3,0) = 1050;
        }

        // F(0,0) = F(0,0) + 100;
        // F(1,0) = F(1,0) + 100;
        // F(0) = std::max(1050.0, std::min(F(0), 1900.0)); // Clamp F(0) between 1050 and 1900
        // F(1) = std::max(1050.0, std::min(F(1), 1900.0)); // Clamp F(1)
        // F(2) = std::max(1050.0, std::min(F(2), 1900.0)); // Clamp F(2)
        // F(3) = std::max(1050.0, std::min(F(3), 1900.0)); // Clamp F(3)

        // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

        // // Get the logger
        // auto logger = this->get_logger();

        // // Stringstream for converting matrices to strings
        // std::stringstream ss;

        // // Assuming F is already computed and ready to print

        // // Printing F (Matrix)
        // ss << F.format(CleanFmt);
        // RCLCPP_INFO(logger, "Matrix F:\n%s", ss.str().c_str());
        // ss.str(""); // Clearing the stringstream
        
        auto msg2 = blimp_interfaces::msg::EscInput();
        msg2.esc_pins = {5,6,13,26};
        msg2.pwm_l = F(1,0); 
        msg2.pwm_r = F(0,0);
        msg2.pwm_u = F(3,0);
        msg2.pwm_d = F(2,0);
        RCLCPP_INFO(this->get_logger(), "M1: %f  M2: %f  M3: %f  M4: %f", msg2.pwm_l, msg2.pwm_r, msg2.pwm_u, msg2.pwm_d);
        publisher_->publish(msg2);
        //RCLCPP_INFO(this->get_logger(), "M1: %f  M2: %f  M3: %f  M4: %f", F(0), F(1), F(2), F(3));

    }


    // Members of the Class

    float Fx1, Fx2, Fx3, Fx4;
    float Fy1, Fy2, Fy3, Fy4;
    float Fz1, Fz2, Fz3, Fz4;
    float lx1, lx2, lx3, lx4;
    float ly1, ly2, ly3, ly4;
    float lz1, lz2, lz3, lz4;
    double f1, f2, f3, f4;
    Eigen::DiagonalMatrix<double, 4> K;
    Eigen::DiagonalMatrix<double, 4> K_inv;
    Eigen::Matrix<double, 6,1> tau;
    Eigen::Matrix<double, 6,4> Q;
    Eigen::Matrix<double, 4,4> T;
    Eigen::Matrix<double, 4,6> Qp;
    Eigen::Matrix<double, 4,1> F;
    Eigen::Matrix<double, 4,1> B;

    rclcpp::Subscription<blimp_interfaces::msg::CartCoord>::SharedPtr subscriber_;
    rclcpp::Publisher<blimp_interfaces::msg::EscInput>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalloonEscInput>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}