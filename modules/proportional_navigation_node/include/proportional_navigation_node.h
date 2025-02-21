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
    void publishSetpoint(const Eigen::Vector3f &velocity, const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos);
    void publishAttitude(float roll, float pitch, float yaw, float thrust);
    void computeStickControl();

    bool setOffboardAndArm();
    float calculateYaw(const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos);

    geometry_msgs::TwistStamped computeProportionalNavigation();
    mavros_msgs::ManualControl computeAttitudeControl();

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

    // Used to store the UAV's position, velocity, and attitude
    Eigen::Vector3f drone_position_;
    Eigen::Vector3f drone_velocity_;
    Eigen::Quaternionf drone_orientation_; // Using Eigen quaternion for attitude representation
    Eigen::Vector3f drone_rpy_;

    double Kp_yaw_, Kp_thrust_, Kp_pitch_roll_;
    double target_center_x_, target_center_y_; // height * width (480 * 640)

    // Low-pass filter parameters
    const double alpha_min_ = 0.1;   // High smoothness, suitable for small errors
    const double alpha_max_ = 0.15;  // Quick response, suitable for large errors
    const double max_error_ = 200.0; // Error normalization threshold

    // Variables to store the control input from the previous frame
    double prev_roll_input_ = 0.0;
    double prev_pitch_input_ = 0.0;
    double prev_yaw_input_ = 0.0;
    double prev_thrust_input_ = 0.5;
};
