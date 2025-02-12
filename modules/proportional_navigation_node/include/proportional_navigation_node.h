#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ManualControl.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <sensor_msgs/Imu.h>

#include "pidController.h"
#include "so3Controller.h"

using namespace Eigen;
using namespace std;

class ProportionalNavigationNode
{
public:
    ProportionalNavigationNode();
    ~ProportionalNavigationNode() = default;
    void run();

private:
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void targetCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void gpsOriginCallback(const geographic_msgs::GeoPointStamped::ConstPtr &msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void boundboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void publishSetpoint(const Eigen::Vector3f &velocity, const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos);
    void publishAttitude(float roll, float pitch, float yaw, float thrust);
    void computeStickControl();
    void computeAttitudeStick();

    bool setOffboardAndArm();
    float calculateYaw(const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos);

    geometry_msgs::TwistStamped computeProportionalNavigation();
    mavros_msgs::ManualControl computeAttitudeControl();

    VectorXd predict(Vector2d imu_acc);
    VectorXd update(Vector2d z);

    ros::NodeHandle nh_;
    ros::Publisher velocity_pub_;
    ros::Publisher setpoint_pub_;
    ros::Publisher attitude_pub_;
    ros::Publisher manual_control_pub_;

    ros::Subscriber state_sub_;
    ros::Subscriber position_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber gps_origin_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber rc_sub_;
    ros::Subscriber bbox_sub_;
    ros::Subscriber imu_sub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_position_;
    sensor_msgs::NavSatFix target_;
    geographic_msgs::GeoPointStamped gps_origin_;
    sensor_msgs::Joy joy_;

    bool joy_updated_{false};
    bool start_attack_{false};

    bool boundingbox_updated_{false};
    darknet_ros_msgs::BoundingBox boundingbox_;

    Eigen::Vector3d global_origin_;

    Eigen::Vector3f target_ned_;
    Eigen::Vector3f target_velocity_;

    // 用于存储无人机的位置、速度和姿态
    Eigen::Vector3f drone_position_;
    Eigen::Vector3f drone_velocity_;
    Eigen::Quaternionf drone_orientation_; // 使用Eigen的四元数表示姿态
    Eigen::Vector3f drone_rpy_;

    float thrust_prev_ = 0.5f;
    float acc_z_;

    double Kp_yaw_, Kp_thrust_, Kp_pitch_roll_;
    double target_center_x_, target_center_y_; // height * width (480 * 640)

    // 低通滤波参数
    const double alpha_min_ = 0.1;   // 平滑性高，适用于小误差
    const double alpha_max_ = 0.15;  // 响应快，适用于大误差
    const double max_error_ = 200.0; // 误差归一化阈值

    // 变量用于存储前一帧的控制输入
    double prev_roll_input_ = 0.0;
    double prev_pitch_input_ = 0.0;
    double prev_yaw_input_ = 0.0;
    double prev_thrust_input_ = 0.5;

    double omega_x_, omega_y_; // 角速度
    double dt = 0.02;
    MatrixXd A = MatrixXd(6, 6);
    MatrixXd B = MatrixXd(6, 2);
    MatrixXd H = MatrixXd(2, 6);
    MatrixXd Q = MatrixXd(6, 6);
    MatrixXd R = MatrixXd(2, 2);
    VectorXd x = VectorXd(6);
    MatrixXd P = MatrixXd(6, 6);

    Matrix3d R_actual_ = Matrix3d::Identity(); // 实际姿态
    Vector3d omega_actual_ = Vector3d::Zero(); // 实际角速度
    Vector2d imu_acc_ = Vector2d::Zero();      // 实际加速度

    // PID 控制器（用于控制目标的角速度）
    // PIDController pid_x_;
    // PIDController pid_y_;
    // SO3Controller so3_;

    Vector3d omega_d_;
};
