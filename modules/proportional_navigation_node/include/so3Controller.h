#include <Eigen/Dense>

using namespace Eigen;

// --- SO(3) 控制器 ---
class SO3Controller
{
public:
    SO3Controller(double kp, double kd)
    {
        K_R = kp;
        K_W = kd;
    }

    Vector3d compute(Vector3d &omega, Matrix3d &R, Matrix3d &R_d)
    {
        // 计算姿态误差 e_R
        Matrix3d error_matrix = 0.5 * (R_d.transpose() * R - R.transpose() * R_d);
        Vector3d e_R(error_matrix(2, 1), error_matrix(0, 2), error_matrix(1, 0));

        // 计算角速度误差 e_W
        Vector3d e_W = omega; // 这里假设 omega_d = 0, 你可以改成 omega_d - omega_actual

        // 计算 SO(3) 控制律
        return -K_R * e_R - K_W * e_W;
    }

private:
    double K_R, K_W; // 姿态控制增益
};