#ifndef OMC_ATTITUDE_CTL_PID_HPP_
#define OMC_ATTITUDE_CTL_PID_HPP_

#include <iostream>

class PID {
public:
    PID() = default;
    PID(double kp, double ki, double kd, double integration_limit, double dt, double lpf_pass_band_freq)
        : kp_(kp), ki_(ki), kd_(kd), integration_limit_(integration_limit), dt_(dt), prev_error_(0), integral_(0), filtered_derivative_(0), lpf_pass_band_freq_(lpf_pass_band_freq) {
        calculateLPFCoefficient();
    }

    void setLPFPassBandFreq(double lpf_pass_band_freq) {
        lpf_pass_band_freq_ = lpf_pass_band_freq;
        calculateLPFCoefficient();
    }

    double compute(double setpoint, double feedback) {
        double error = setpoint - feedback;

        // Proportional term
        double P = kp_ * error;

        // Integral term with saturation
        integral_ += ki_ * error * dt_;
        if (integral_ > integration_limit_) {
            integral_ = integration_limit_;
        } else if (integral_ < -integration_limit_) {
            integral_ = -integration_limit_;
        }
        double I = integral_;

        // Derivative term with low-pass filter
        double derivative = (error - prev_error_) / dt_;
        filtered_derivative_ = alpha_ * filtered_derivative_ + (1.0 - alpha_) * derivative;
        double D = kd_ * filtered_derivative_;

        // PID control output
        double output = P + I + D;

        // Update previous error for the next iteration
        prev_error_ = error;

        return output;
    }

private:
    double kp_; // Proportional gain
    double ki_; // Integral gain
    double kd_; // Derivative gain
    double integration_limit_; // Integral saturation limit
    double dt_; // Time step
    double prev_error_; // Previous error for computing the derivative
    double integral_; // Integral term
    double filtered_derivative_; // Filtered derivative term

    double lpf_pass_band_freq_; // LPF pass band frequency
    double alpha_; // LPF coefficient

    void calculateLPFCoefficient() {
        // Calculate LPF coefficient based on pass band frequency
        alpha_ = 1.0 / (1.0 + 2.0 * 3.141592653589793 * lpf_pass_band_freq_ * dt_);
    }
};

#endif // OMC_ATTITUDE_CTL_PID_HPP_