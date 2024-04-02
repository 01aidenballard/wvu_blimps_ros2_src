#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/imu_data.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/baro_data.hpp"
#include "blimp_interfaces/msg/esc_input.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

class DynamicModel : public rclcpp::Node {
public:
    DynamicModel()
    : Node("dynamic_model"),
    height{0.0},
    m{0.42511728},
    zg{0.21788162}
    {
        this->declare_parameter<double>("rho_air", 1.225); // kg/m^3
        this->declare_parameter<double>("buoyancy", 0.42511728);

        rho_air = this->get_parameter("rho_air").as_double();
        B = this->get_parameter("buoyancy").as_double();
        W = m * 9.81;
        //        a     b      density    Ix         Iy        Iz
        M_matrix(0.86,0.28,rho_air,0.058241,0.08583106,0.04598382);

        D_matrix();

        R_imu << 1, 0, 0,
                  0,-1, 0,
                  0, 0,-1;


        subscriber_imu = this->create_subscription<blimp_interfaces::msg::ImuData>(
            "imu_data", 10, std::bind(&DynamicModel::callback_imu, this, std::placeholders::_1));

        subscriber_baro = this->create_subscription<blimp_interfaces::msg::BaroData>(
            "barometer_data", 10, std::bind(&DynamicModel::callback_baro, this, std::placeholders::_1));

        subscriber_dynamic_model = this->create_subscription<blimp_interfaces::msg::CartCoord>(
            "balloon_input", 10, std::bind(&DynamicModel::callback_dynamic_model, this, std::placeholders::_1));

        kine_publisher = this->create_publisher<blimp_interfaces::msg::CartCoord>("forces", 10);
        RCLCPP_INFO(this->get_logger(), "Dynamics are modeled");

        // ... (initialize matrices here as needed) ...
        time(&start);
    }

    void M_matrix(double a, double b, double rho, double Ix, double Iy, double Iz) {
        double e = std::sqrt((1 - (b/a) * (b/a)));
        double beta0 = (1 / (e * e)) - ((std::log((1 + e) / (1 - e)) * ((1 - e * e) / (2 * e * e * e))));
        double alpha0 = ((2 * (1 - e * e)) / (e * e * e)) * ((0.5 * std::log((1 + e) / (1 - e))) - e);
        double kprime = (e * e * e * e * (beta0 - alpha0)) / ((2 - e * e) * (2 * e * e - (2 - e * e) * (beta0 - alpha0)));
        double k1 = alpha0 / (2 - alpha0);
        double k2 = beta0 / (2 - beta0);
        double Izh = (4/5) * M_PI * rho * a * b * b * (a * a + b * b);
        mx_prime = m + k1 * m;
        my_prime = m + k2 * m;
        mz_prime = m + k2 * m;
        Ix_prime_ = Ix;
        Iy_prime_ = Iy + kprime * Izh;
        Iz_prime_ = Iz + kprime * Izh;

        M << mx_prime, 0, 0, 0, m * zg, 0,
        0, my_prime, 0, -m * zg, 0, 0,
        0, 0, mz_prime, 0, 0, 0,
        0, -m * zg, 0, Ix_prime_, 0, 0,
        m * zg, 0, 0, 0, Iy_prime_, 0,
        0, 0, 0, 0, 0, Iz_prime_;
    }

    void D_matrix() {
        D = Eigen::DiagonalMatrix<double, 6>(-0.0254, 0.0771, 0.0216, -0.1820, 0.3280, 0.0116);
    }

private:
    // ... (function definitions to compute M and D matrices) ...

    void callback_baro(const blimp_interfaces::msg::BaroData::SharedPtr msg) {
        height = msg->height;
    }

    void callback_imu(const blimp_interfaces::msg::ImuData::SharedPtr msg) {

        lin_accel << msg->imu_lin_accel[0], msg->imu_lin_accel[1], msg->imu_lin_accel[2];
        euler << msg->imu_euler[0], msg->imu_euler[1], msg->imu_euler[2];
        gyro << msg->imu_gyro[0], msg->imu_gyro[1], msg->imu_gyro[2];

        lin_accel = R_imu * lin_accel;
        euler = R_imu * euler;
        gyro = R_imu * gyro;        
    }

    void callback_dynamic_model(const blimp_interfaces::msg::CartCoord::SharedPtr msg) {
        time(&finish);
        double dt = difftime(finish, start);
        time(&start);
        vz = (height_old - height)/dt;
        vx = 1.0;
        vy = 0.0;
        vel << vx, vy, vz, gyro(0), gyro(1), gyro(2);
        height_old = height;

        accel << msg->x,msg->y,msg->z,msg->theta,msg->phi,msg->psy;
        
        C << 0, 0, 0, 0, -(mz_prime * vz) - (my_prime * vy - m * zg * gyro(0)), 0,
            0, 0, 0, -(mz_prime * vz), 0, -(mx_prime * vx - m * zg * gyro(1)),
            0, 0, 0, -(my_prime * vy - m * zg * gyro(0)), -(mx_prime * vx + m * zg * gyro(1)), 0,
            0, -(mz_prime * vz), -(my_prime * vy - m * zg * gyro(0)), 0, -(Iz_prime_ * gyro(2)), -(m * zg * vx + Iy_prime_ * gyro(1)),
            -(mz_prime * vz), 0, -(mx_prime * vx - m * zg * gyro(1)), -(Iz_prime_ * gyro(2)), 0, -(m * zg * vy - Ix_prime_ * gyro(0)),
            -(my_prime * vy - m * zg * gyro(0)), -(mx_prime * vx + m * zg * gyro(1)), 0, -(m * zg * vx - Iy_prime_ * gyro(1)), -(m * zg * vy + Ix_prime_ * gyro(0)), 0;
        

        g << (W - B) * sin(euler(1)),
            (W - B) * cos(euler(1)) * sin(euler(0)),
            (W - B) * cos(euler(1)) * cos(euler(0)),
            -zg * W * cos(euler(1)) * sin(euler(0)),
            -zg * W * sin(euler(1)),
            0;       
        
        tau = M*accel + D*vel + C*vel + g;

        auto msg2 = blimp_interfaces::msg::CartCoord();
        msg2.x = tau(0);
        msg2.y = tau(1);
        msg2.z = tau(2);
        msg2.theta = tau(3);
        msg2.phi = tau(4);
        msg2.psy = tau(5);
        kine_publisher->publish(msg2);
        RCLCPP_INFO(this->get_logger(), "Start time: %ld", start);
    }



    // Members for the node
    double height;
    double m;
    double zg;
    double rho_air;
    double B;
    double W;
    double mx_prime, my_prime, mz_prime;
    double Ix_prime_, Iy_prime_, Iz_prime_;
    Eigen::DiagonalMatrix<double, 6> D; // 6x6 diagonal matrix
    Eigen::Matrix<double, 6, 6> M; // 6x6 matrix
    Eigen::Matrix3d R_imu; // Replace the rotation matrix defined in Python
    time_t start, finish;
    double vx;
    double vy;
    double vz;
    double height_old;
    Eigen::Matrix<double, 6, 1> tau;
    Eigen::Matrix<double, 6, 1> g;
    Eigen::Matrix<double, 6, 6> C;
    Eigen::Vector3d gyro;
    Eigen::Matrix<double, 6,1> vel;
    Eigen::Matrix<double, 6,1> accel;
    Eigen::Vector3d lin_accel;
    Eigen::Vector3d euler;
    rclcpp::Subscription<blimp_interfaces::msg::ImuData>::SharedPtr subscriber_imu;
    rclcpp::Subscription<blimp_interfaces::msg::BaroData>::SharedPtr subscriber_baro;
    rclcpp::Subscription<blimp_interfaces::msg::CartCoord>::SharedPtr subscriber_dynamic_model;
    rclcpp::Publisher<blimp_interfaces::msg::CartCoord>::SharedPtr kine_publisher;
    // ... (other member variables and matrices) ...
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicModel>());
    rclcpp::shutdown();
    // Use 'std::system' if you really need to call "sudo killall pigpiod", but consider the security implications
    return 0;
}