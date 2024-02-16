#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <OsqpEigen/Solver.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <OsqpEigen/OsqpEigen.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "omc_attitude_ctl/pid.hpp"

using namespace std::chrono_literals;

class AttitudeController : public rclcpp::Node {
public:
    AttitudeController() : Node("attitude_controller") {
        // Create publisher and timer thread to publish message
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&AttitudeController::timer_callback, this));

        // Create subscriber
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/omc/odom", 10, std::bind(&AttitudeController::subscribe_callback, this, std::placeholders::_1));

        // Initialize data
        position_ = std::make_shared<geometry_msgs::msg::Vector3>();
        orientation_ = tf2::Quaternion(0, 0, 0, 1);

        // Controller initialize
        position_controller[0] = PID(2000.f, 10.0f, 200.f, 200.0, 0.010, 100.0);
        position_controller[1] = PID(2000.f, 10.0f, 200.f, 200.0, 0.010, 100.0);
        position_controller[2] = PID(2000.f, 10.0f, 200.f, 200.0, 0.010, 100.0);

        attitude_controller[0] = PID(100.0f, 0.0f, 0.0f, 200.0, 0.010, 100.0);
        attitude_controller[1] = PID(100.0f, 0.0f, 0.0f, 200.0, 0.010, 100.0);
        attitude_controller[2] = PID(100.0f, 0.0f, 0.0f, 200.0, 0.010, 100.0);


        // OSQP solver initialize
        auto constraints_number = 6;
        auto variables_number = 8;
        
        hessian_.resize(variables_number, variables_number);
        gradient_.resize(variables_number);
        linear_.resize(variables_number, constraints_number);
        boundary_.resize(constraints_number);

        solver_.settings()->setWarmStart(true);
        solver_.settings()->setVerbosity(false);

        hessian_.setZero();
        gradient_.setZero();
        linear_.setZero();
        boundary_.setZero();

        hessian_.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1;

        linear_.insert(0, 0) = -1;
        linear_.insert(0, 1) = -1;
        linear_.insert(0, 2) = 1;
        linear_.insert(0, 3) = -1;
        linear_.insert(0, 4) = -1;
        linear_.insert(0, 5) = 1;

        linear_.insert(1, 0) = 1;
        linear_.insert(1, 1) = 1;
        linear_.insert(1, 2) = 1;
        linear_.insert(1, 3) = 1;
        linear_.insert(1, 4) = 1;
        linear_.insert(1, 5) = 1;

        linear_.insert(2, 0) = 1;
        linear_.insert(2, 1) = -1;
        linear_.insert(2, 2) = -1;
        linear_.insert(2, 3) = 1;
        linear_.insert(2, 4) = -1;
        linear_.insert(2, 5) = -1;

        linear_.insert(3, 0) = -1;
        linear_.insert(3, 1) = 1;
        linear_.insert(3, 2) = -1;
        linear_.insert(3, 3) = -1;
        linear_.insert(3, 4) = 1;
        linear_.insert(3, 5) = -1;

        linear_.insert(4, 0) = 1;
        linear_.insert(4, 1) = 1;
        linear_.insert(4, 2) = 1;
        linear_.insert(4, 3) = -1;
        linear_.insert(4, 4) = -1;
        linear_.insert(4, 5) = -1;

        linear_.insert(5, 0) = -1;
        linear_.insert(5, 1) = -1;
        linear_.insert(5, 2) = 1;
        linear_.insert(5, 3) = 1;
        linear_.insert(5, 4) = 1;
        linear_.insert(5, 5) = -1;

        linear_.insert(6, 0) = -1;
        linear_.insert(6, 1) = 1;
        linear_.insert(6, 2) = -1;
        linear_.insert(6, 3) = 1;
        linear_.insert(6, 4) = -1;
        linear_.insert(6, 5) = 1;

        linear_.insert(7, 0) = 1;
        linear_.insert(7, 1) = -1;
        linear_.insert(7, 2) = -1;
        linear_.insert(7, 3) = -1;
        linear_.insert(7, 4) = 1;
        linear_.insert(7, 5) = 1;

        solver_.data()->setNumberOfVariables(variables_number);
        solver_.data()->setNumberOfConstraints(constraints_number);
        solver_.data()->setHessianMatrix(hessian_);
        solver_.data()->setGradient(gradient_);
        solver_.data()->setLinearConstraintsMatrix(linear_.transpose());
        solver_.data()->setBounds(boundary_, boundary_);
        solver_.initSolver();
    }

private:
    
    void timer_callback() {
        // Check if odom_data_ availablility
        if (isnan(position_->x) || isnan(position_->y) || isnan(position_->z)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid data: position_ -> geometry_msgs::msg::Vector3");
            return;
        }

        if (isnan(orientation_.getW()) || isnan(orientation_.getX()) ||
            isnan(orientation_.getY()) || isnan(orientation_.getZ())) {
            RCLCPP_ERROR(this->get_logger(), "Invalid data: orientation_ -> tf2::Quaternion");
            return;
        }

        auto world_force = tf2::Vector3(
            position_controller[0].compute(2.0, position_->x),
            position_controller[1].compute(2.0, position_->y),
            position_controller[2].compute(2.0, position_->z));

        auto body_force = tf2::quatRotate(orientation_.inverse(), world_force);


        auto target_quat = tf2::Quaternion(0, 0, 0, 1);
        auto error_quat = target_quat * orientation_.inverse();
        auto target_angular_rate = 100*2*acos(error_quat.getW()) / sqrt(1.000001-pow(error_quat.getW(), 2))*error_quat.getAxis();
        // tf2::Matrix3x3 convert_angle(orientation_);
        // std::array<double, 3> eular_angle;
        // convert_angle.getRPY(eular_angle[0], eular_angle[1], eular_angle[2]);

        // auto target_torque = tf2::Vector3(
        //     attitude_controller[0].compute(.0f, eular_angle[2]),
        //     attitude_controller[1].compute(.0f, eular_angle[1]),
        //     attitude_controller[2].compute(.0f, eular_angle[0])
        // );
        
        // RCLCPP_INFO(this->get_logger(), "pos: %f, %f, %f", position_->x, position_->y, position_->z);
        // RCLCPP_INFO(this->get_logger(), "wf: %f, %f, %f", world_force.getX(), world_force.getY(), world_force.getZ());
        // RCLCPP_INFO(this->get_logger(), "bf: %f, %f, %f", body_force.getX(), body_force.getY(), body_force.getZ());
        // RCLCPP_INFO(this->get_logger(), "orientation: %f, %f, %f, %f", orientation_.getW(), orientation_.getX(), orientation_.getY(), orientation_.getZ());

        boundary_ << body_force.getX()*2, body_force.getY()*2, body_force.getZ()*2
        ,target_angular_rate.getX(), target_angular_rate.getY(), target_angular_rate.getZ();
        // ,0,0,0;
        solver_.updateBounds(boundary_, boundary_);
        solver_.solveProblem();

        auto motor_force = solver_.getSolution();
        RCLCPP_INFO(this->get_logger(), "TargetTorque: %2f %2f %2f", target_angular_rate.getX(), target_angular_rate.getY(),
                    target_angular_rate.getZ());
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {
            motor_force(0),
            motor_force(1),
            motor_force(2),
            motor_force(3),
            // motor_force(1),
            // motor_force(0),
            // motor_force(3),
            // motor_force(2)
            motor_force(4),
            motor_force(5),
            motor_force(6),
            motor_force(7)
        };

        publisher_->publish(message);
    }

    void subscribe_callback(std::shared_ptr<nav_msgs::msg::Odometry> pData) {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        position_->set__x(pData->pose.pose.position.x);
        position_->set__y(pData->pose.pose.position.y);
        position_->set__z(pData->pose.pose.position.z);
        orientation_.setW(pData->pose.pose.orientation.w);
        orientation_.setX(pData->pose.pose.orientation.x);
        orientation_.setY(pData->pose.pose.orientation.y);
        orientation_.setZ(pData->pose.pose.orientation.z);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    // Feedback data
    std::shared_ptr<geometry_msgs::msg::Vector3> position_{nullptr};
    tf2::Quaternion orientation_{};
    std::mutex mutex_lock_;

    PID position_controller[3];
    PID attitude_controller[3];

    OsqpEigen::Solver solver_;
    Eigen::SparseMatrix<double> hessian_;
    Eigen::SparseMatrix<double> linear_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd boundary_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AttitudeController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}