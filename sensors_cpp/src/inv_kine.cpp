#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/imu_data.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/baro_data.hpp"
#include "blimp_interfaces/msg/esc_input.hpp"
#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;

class DynamicModel : public rclcpp::Node {
public:
    DynamicModel()
    : Node("dynamic_model"),
    height(0.0),
    m(0.42511728),
    zg(0.21788162)
    {
        this->declare_parameter<double>("rho_air", 1.225); // kg/m^3
        this->declare_parameter<double>("buoyancy", 0.42511728);

        rho_air = this->get_parameter("rho_air").as_double();
        B = this->get_parameter("buoyancy").as_double();
        
        //        a     b    c  density    Ix         Iy        Iz
        M_matrix(0.86,0.28,0.38,rho_air,0.058241,0.08583106,0.04598382);

        D_matrix()


        subscriber_imu_ = this->create_subscription<blimp_interfaces::msg::ImuData>(
            "imu_data", 10, std::bind(&DynamicModel::callback_imu, this, std::placeholders::_1));

        subscriber_baro_ = this->create_subscription<blimp_interfaces::msg::BaroData>(
            "barometer_data", 10, std::bind(&DynamicModel::callback_baro, this, std::placeholders::_1));

        subscriber_dynamic_model_ = this->create_subscription<blimp_interfaces::msg::EscInput>(
            "ESC_input", 10, std::bind(&DynamicModel::callback_dynamic_model, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dynamics are modeled");

        // ... (initialize matrices here as needed) ...
    }

    void DynamicModel::M_matrix(double a, double b, double c, double rho, double Ix, double Iy, double Iz) {
        double e = std::sqrt((1 - (b/a) * (b/a)));
        double beta0 = (1 / (e * e)) - ((std::log((1 + e) / (1 - e)) * ((1 - e * e) / (2 * e * e * e))));
        double alpha0 = ((2 * (1 - e * e)) / (e * e * e)) * ((0.5 * std::log((1 + e) / (1 - e))) - e);
        double kprime = (e * e * e * e * (beta0 - alpha0)) / ((2 - e * e) * (2 * e * e - (2 - e * e) * (beta0 - alpha0)));
        double k1 = alpha0 / (2 - alpha0);
        double k2 = beta0 / (2 - beta0);
        double Izh = (4/5) * M_PI * rho * a * b * b * (a * a + b * b);
        mx_prime_ = m + k1 * m;
        my_prime_ = m + k2 * m;
        mz_prime_ = m + k2 * m;
        Ix_prime_ = Ix;
        Iy_prime_ = Iy + kprime * Izh;
        Iz_prime_ = Iz + kprime * Izh;

        M_ << mx_prime_, 0, 0, 0, m * zg_, 0,
        0, my_prime_, 0, -m * zg_, 0, 0,
        0, 0, mz_prime_, 0, 0, 0,
        0, -m * zg_, 0, Ix_prime_, 0, 0,
        m * zg_, 0, 0, 0, Iy_prime_, 0,
        0, 0, 0, 0, 0, Iz_prime_;
    }

    void DynamicModel::D_matrix() {
        D_ = Eigen::DiagonalMatrix<double, 6>(-0.0254, 0.0771, 0.0216, -0.1820, 0.3280, 0.0116);
    }

    // Member variables related to matrices in the DynamicModel class:
    Eigen::MatrixXd M_; // 6x6 matrix
    Eigen::DiagonalMatrix<double, 6> D_; // 6x6 diagonal matrix

    // Additionally, we will need to declare other variables used such as:
    double mx_prime_, my_prime_, mz_prime_;
    double Ix_prime_, Iy_prime_, Iz_prime_;
    double m;  // mass of the balloon
    double zg_;  // height of the center of gravity above the origin of the body frame
    // and so on...

private:
    // ... (function definitions to compute M and D matrices) ...

    void callback_baro(const blimp_interfaces::msg::BaroData::SharedPtr msg) {
        height = msg->height;
    }

    void callback_imu(const blimp_interfaces::msg::ImuData::SharedPtr msg) {
        // Conversion of Python's list to Eigen's vector
        Eigen::Vector3d euler(msg->imu_euler.x, msg->imu_euler.y, msg->imu_euler.z);
        // ... (the rest of your callback_imu function using Eigen instead of NumPy) ...
    }

    void callback_dynamic_model(const blimp_interfaces::msg::EscInput::SharedPtr msg) {
        // ... (the rest of your callback_dynamic_model function using Eigen instead of NumPy) ...
    }

    // Members for the node
    double height;
    double m;
    double zg;
    double rho_air;
    double B;
    Eigen::Matrix3d R_imu_; // Replace the rotation matrix defined in Python
    rclcpp::Subscription<blimp_interfaces::msg::ImuData>::SharedPtr subscriber_imu_;
    rclcpp::Subscription<blimp_interfaces::msg::BaroData>::SharedPtr subscriber_baro_;
    rclcpp::Subscription<blimp_interfaces::msg::EscInput>::SharedPtr subscriber_dynamic_model_;
    // ... (other member variables and matrices) ...
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicModel>());
    rclcpp::shutdown();
    // Use 'std::system' if you really need to call "sudo killall pigpiod", but consider the security implications
    return 0;
}