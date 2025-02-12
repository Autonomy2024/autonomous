
#include <ros/ros.h>

// --- PID 控制器 ---
class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        prev_error = 0;
        integral = 0;
    }

    double compute(double error)
    {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

private:
    double Kp, Ki, Kd;
    double prev_error, integral;
    double dt = 0.02; // 50Hz 控制
};
