#include "proportional_navigation_node.h"

// 定义常量
#define GRAVITY 9.81f
#define MAX_TILT_ANGLE 0.174f              // 10度，最大倾斜角度
#define MAN_TILT_MAX (35.f / (180 / M_PI)) // Maximal tilt angle in manual or altitude mode
#define MAN_YAW_MAX (150.f / (180 / M_PI)) // Max manual yaw rate

// 将航向角规范化到 [-π, π]
double normalizeYaw(double yaw)
{
    // 使用模运算将角度限制在 [-π, π]
    yaw = fmod(yaw + M_PI, 2 * M_PI);
    if (yaw < 0)
    {
        yaw += 2 * M_PI;
    }
    return yaw - M_PI;
}
/**********************************************************
 *@File name:map_projection_project
 *@param[ref_lat]: double ，原点lat坐标
 *@param[ref_lon]: double，原点lon坐标
 *@param[lat]: double，相对lat坐标
 *@param[lon]: double，相对lon坐标
 *@param[x]: float，转换的enu的x坐标
 *@param[y]: float，转换的enu的y坐标
 *@Description:坐标转换
 **********************************************************/
void map_projection_project(double ref_lat, double ref_lon, double lat,
                            double lon, float &x, float &y)
{
    static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;

    double ref_lat_d = ref_lat * M_PI / 180;
    double ref_lon_d = ref_lon * M_PI / 180;
    double lat_d = lat * M_PI / 180;
    double lon_d = lon * M_PI / 180;
    double cos_d_lon = cos(lon_d - ref_lon_d);
    double arg = sin(ref_lat_d) * sin(lat_d) + cos(ref_lat_d) * cos(lat_d) * cos_d_lon;
    arg = std::max(std::min(arg, 1.0), -1.0);
    double c = acos(arg);
    double k = 1;
    if (abs(c) > 0)
    {
        k = c / sin(c);
    }
    y = k * (cos(ref_lat_d) * sin(lat_d) - sin(ref_lat_d) * cos(lat_d) * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    x = k * (cos(lat_d) * sin(lon_d - ref_lon_d)) * CONSTANTS_RADIUS_OF_EARTH;
}

/*
 * Constrain the 3D vector given a maximum XY norm
 * If the XY norm of the 3D vector is larger than the maximum norm, the whole vector
 * is scaled down to respect the constraint.
 * If the maximum norm is small (defined by the "accuracy" parameter),
 * only the XY components are scaled down to avoid affecting
 * Z in case of numerical issues
 */
inline void clampToXYNorm(Eigen::Vector3f &target, float max_xy_norm, float accuracy = 0.01f)
{
    const float xynorm = target.head<2>().norm();
    const float scale_factor = (xynorm > FLT_EPSILON)
                                   ? max_xy_norm / xynorm
                                   : 1.f;

    if (scale_factor < 1.f)
    {
        if (max_xy_norm < accuracy && xynorm < accuracy)
        {
            target.head<2>() = target.head<2>() * scale_factor;
        }
        else
        {
            target *= scale_factor;
        }
    }
}

/*
 * Constrain the 3D vector given a maximum Z norm
 * If the Z component of the 3D vector is larger than the maximum norm, the whole vector
 * is scaled down to respect the constraint.
 * If the maximum norm is small (defined by the "accuracy parameter),
 * only the Z component is scaled down to avoid affecting
 * XY in case of numerical issues
 */
inline void clampToZNorm(Eigen::Vector3f &target, float max_z_norm, float accuracy = 0.01f)
{
    const float znorm = fabs(target(2));
    const float scale_factor = (znorm > FLT_EPSILON)
                                   ? max_z_norm / znorm
                                   : 1.f;

    if (scale_factor < 1.f)
    {
        if (max_z_norm < accuracy && znorm < accuracy)
        {
            target(2) *= scale_factor;
        }
        else
        {
            target *= scale_factor;
        }
    }
}

// NED坐标系转换函数
Eigen::Vector3f enuToNed(const Eigen::Vector3f &enu)
{
    // ENU到NED坐标系转换的旋转矩阵
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix << 0, 1, 0,
        1, 0, 0,
        0, 0, -1;

    return rotation_matrix * enu;
}

ProportionalNavigationNode::ProportionalNavigationNode()
{
    // 初始化发布器和订阅器
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("iris_0/mavros/setpoint_velocity/cmd_vel", 10);
    state_sub_ = nh_.subscribe("iris_0/mavros/state", 10, &ProportionalNavigationNode::stateCallback, this);
    position_sub_ = nh_.subscribe("iris_0/mavros/local_position/pose", 10, &ProportionalNavigationNode::positionCallback, this);
    // 订阅 odom 信息
    odom_sub_ = nh_.subscribe("iris_0/mavros/local_position/odom", 10, &ProportionalNavigationNode::odomCallback, this);
    rc_sub_ = nh_.subscribe("joy", 10, &ProportionalNavigationNode::joyCallback, this);
    bbox_sub_ = nh_.subscribe("LightTrack/bounding_boxes", 10, &ProportionalNavigationNode::boundboxCallback, this);

    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("iris_0/mavros/setpoint_raw/local", 10);
    attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("iris_0//mavros/setpoint_raw/attitude", 10);
    manual_control_pub_ = nh_.advertise<mavros_msgs::ManualControl>("/iris_0/mavros/manual_control/send", 10);

    target_sub_ = nh_.subscribe("rover_1/mavros/global_position/global", 10, &ProportionalNavigationNode::targetCallback, this);
    gps_origin_sub_ = nh_.subscribe("iris_0/mavros/global_position/gp_origin", 10, &ProportionalNavigationNode::gpsOriginCallback, this);

    // 服务客户端，用于设置模式和解锁
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("iris_0/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("iris_0/mavros/cmd/arming");

    target_ned_.Zero();
    target_ned_(2) = -0.8f;

    // Control gains
    Kp_yaw_ = 0.5;        // Proportional gain for yaw
    Kp_thrust_ = 0.1;     // Proportional gain for thrust
    Kp_pitch_roll_ = 0.1; // Proportional gain for pitch/roll

    target_center_x_ = 320;
    target_center_y_ = 240;
}

void ProportionalNavigationNode::run()
{
    ros::Rate rate(50); // 50 Hz 控制频率

    while (ros::ok() && !current_state_.connected)
    {
        ROS_INFO_THROTTLE(1, "Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }

    // 设置初始目标速度，以维持 OFFBOARD 模式
    geometry_msgs::TwistStamped initial_cmd;
    initial_cmd.twist.linear.x = 0.0;
    initial_cmd.twist.linear.y = 0.0;
    initial_cmd.twist.linear.z = 0.0;

    for (int i = 0; ros::ok() && i < 100; ++i)
    {
        velocity_pub_.publish(initial_cmd); // 发布初始指令
        ros::spinOnce();
        rate.sleep();
    }

    // // 请求切换到 OFFBOARD 模式并解锁
    // if (!setOffboardAndArm())
    // {
    //     ROS_ERROR("Failed to set OFFBOARD mode or arm the drone!");
    //     return;
    // }

    while (ros::ok())
    {
        if (start_attack_)
        {
            // 检查当前状态
            if (current_state_.mode != "ALTCTL" || !current_state_.armed)
            {
                ROS_WARN_THROTTLE(1, "OFFBOARD mode or arming lost. Trying to re-engage...");
                if (!setOffboardAndArm())
                {
                    ROS_ERROR("Re-engagement failed. Exiting...");
                    return;
                }
            }

            // mavros_msgs::ManualControl manual_control_msg = computeAttitudeControl();
            // manual_control_pub_.publish(manual_control_msg);

            computeStickControl();

            // STEP: Use vel
            // 计算并发布速度指令
            // geometry_msgs::TwistStamped cmd_vel = computeProportionalNavigation();
            // velocity_pub_.publish(cmd_vel);

            // Eigen::Vector3f current_pos(current_position_.pose.position.x, current_position_.pose.position.y, current_position_.pose.position.z);
            // Eigen::Vector3f velocity_sp(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z);

            // publishSetpoint(velocity_sp, current_pos, target_ned_);
        }
        else
        {

            mavros_msgs::ManualControl manual_control_msg;
            manual_control_msg.header.stamp = ros::Time::now();

            // 将计算出的roll, pitch, yaw, thrust 转换为手动控制信号
            if (joy_updated_)
            {
                manual_control_msg.x = joy_.axes[4] * 1000.f;               // 左右摇杆（roll）
                manual_control_msg.y = -joy_.axes[3] * 1000.f;              // 前后摇杆（pitch
                manual_control_msg.z = (joy_.axes[1] + 1.f) / 2.f * 1000.f; // 油门（thrust）
                manual_control_msg.r = -joy_.axes[0] * 1000.f;              // 偏航（yaw)
                manual_control_pub_.publish(manual_control_msg);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void ProportionalNavigationNode::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_ = *msg;
}

void ProportionalNavigationNode::positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_position_ = *msg;
}

void ProportionalNavigationNode::targetCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    target_ = *msg;
}

void ProportionalNavigationNode::gpsOriginCallback(const geographic_msgs::GeoPointStamped::ConstPtr &msg)
{
    gps_origin_ = *msg;
}

void ProportionalNavigationNode::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    joy_ = *msg;
    joy_updated_ = true;
    if (joy_.buttons[0] > 0.5f)
    {
        start_attack_ = true;
    }
}

void ProportionalNavigationNode::boundboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    darknet_ros_msgs::BoundingBoxes BoundingBoxes = *msg;
    if (BoundingBoxes.bounding_boxes.size() > 0)
    {
        boundingbox_updated_ = true;
        boundingbox_ = BoundingBoxes.bounding_boxes.at(0);

        std::cout << "boundingbox_.xmin: " << boundingbox_.xmin << "boundingbox_.ymin: "
                  << boundingbox_.ymin << "boundingbox_.xmax: " << boundingbox_.xmax << "boundingbox_.ymax: " << boundingbox_.ymax << std::endl;
    }
}

void ProportionalNavigationNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 提取ENU坐标系下的位置 (从msg.pose.pose.position)
    Eigen::Vector3f enu_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // 将ENU坐标系位置转换到NED坐标系
    drone_position_ = enuToNed(enu_position);

    // 提取ENU坐标系下的速度 (从msg.twist.twist.linear)
    Eigen::Vector3f enu_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    // 将ENU坐标系速度转换到NED坐标系
    drone_velocity_ = enuToNed(enu_velocity);

    // 提取姿态
    drone_orientation_ = Eigen::Quaternionf(msg->pose.pose.orientation.w,
                                            msg->pose.pose.orientation.x,
                                            msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z);

    // tf2::Quaternion q;
    // q.setW(msg->pose.pose.orientation.w);
    // q.setX(msg->pose.pose.orientation.x);
    // q.setY(msg->pose.pose.orientation.y);
    // q.setZ(msg->pose.pose.orientation.z);

    geometry_msgs::Quaternion geo_quat = msg->pose.pose.orientation;
    tf2::Quaternion q(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w);

    // 确保四元数非零
    if (q.length2() < 1e-6)
    {
        ROS_WARN("Received zero quaternion! Skipping conversion.");
        return;
    }

    tf2::Matrix3x3 rotation_matrix(q);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    drone_rpy_ << roll, pitch, yaw;
    // 输出欧拉角（单位：弧度）
    // ROS_INFO("Yaw: %f, Pitch: %f, Roll: %f", yaw, -pitch, roll);
}

bool ProportionalNavigationNode::setOffboardAndArm()
{
    // 设置 OFFBOARD 模式
    mavros_msgs::SetMode set_mode_req;
    set_mode_req.request.custom_mode = "ALTCTL";

    if (set_mode_client_.call(set_mode_req) && set_mode_req.response.mode_sent)
    {
        ROS_INFO("ALTCTL mode enabled");
    }
    else
    {
        ROS_ERROR("Failed to set ALTCTL mode");
        return false;
    }

    // // 解锁飞控
    // mavros_msgs::CommandBool arm_req;
    // arm_req.request.value = true;

    // if (arming_client_.call(arm_req) && arm_req.response.success)
    // {
    //     ROS_INFO("Drone armed");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to arm the drone");
    //     return false;
    // }

    return true;
}

geometry_msgs::TwistStamped ProportionalNavigationNode::computeProportionalNavigation()
{
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();

    // 无人机当前位置
    double ux = current_position_.pose.position.x;
    double uy = current_position_.pose.position.y;
    double uz = current_position_.pose.position.z;

    map_projection_project(gps_origin_.position.latitude, gps_origin_.position.longitude, target_.latitude, target_.longitude, target_ned_.x(), target_ned_.y());

    // 目标点位置
    double tx = target_ned_.x();
    double ty = target_ned_.y();
    double tz = target_ned_.z();

    // 计算相对位置和距离
    double dx = tx - ux;
    double dy = ty - uy;
    double dz = tz - uz;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    // 简化视线角变化率的比例引导 (假设目标静止)
    double N = 4.0; // 导航比
    double lambda_dot_x = dx / distance;
    double lambda_dot_y = dy / distance;

    cmd_vel.twist.linear.x = N * lambda_dot_x;
    cmd_vel.twist.linear.y = N * lambda_dot_y;
    cmd_vel.twist.linear.z = 0.5 * dz; // 简单 P 控制 Z 方向

    // 限制最大速度
    double max_xy_vel = 5.0;
    double max_z_vel = 3.0;
    Eigen::Vector3f cur_pos(current_position_.pose.position.x, current_position_.pose.position.y, current_position_.pose.position.z);
    Eigen::Vector3f target_pos(target_ned_.x(), target_ned_.y(), target_ned_.z());
    Eigen::Vector3f pos_to_target = (target_pos - cur_pos).normalized();
    Eigen::Vector3f vel_sp_constrained = pos_to_target * (max_xy_vel * max_xy_vel + max_z_vel * max_z_vel);
    clampToXYNorm(vel_sp_constrained, max_xy_vel, 0.1f);
    clampToZNorm(vel_sp_constrained, max_z_vel, 0.1f);

    float vel_xy_norm = sqrtf(cmd_vel.twist.linear.x * cmd_vel.twist.linear.x + cmd_vel.twist.linear.y * cmd_vel.twist.linear.y);
    float vel_z_norm = fabsf(cmd_vel.twist.linear.z);

    if (vel_xy_norm > vel_sp_constrained.head<2>().norm())
    {
        cmd_vel.twist.linear.x *= vel_sp_constrained.head<2>().norm() / vel_xy_norm;
        cmd_vel.twist.linear.y *= vel_sp_constrained.head<2>().norm() / vel_xy_norm;
    }

    if (vel_z_norm > fabsf(vel_sp_constrained.z()))
    {
        cmd_vel.twist.linear.z *= fabsf(vel_sp_constrained.z()) / vel_z_norm;
    }

    return cmd_vel;
}

// 定义函数计算目标航向角
float ProportionalNavigationNode::calculateYaw(const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos)
{
    return std::atan2(target_pos.y() - current_pos.y(), target_pos.x() - current_pos.x());
}

// 更新发布航向的逻辑
void ProportionalNavigationNode::publishSetpoint(const Eigen::Vector3f &velocity, const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos)
{
    mavros_msgs::PositionTarget position_target_msg;

    // 设置消息头
    position_target_msg.header.stamp = ros::Time::now();
    position_target_msg.header.frame_id = "map";

    // 设置坐标系
    position_target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 设置控制模式
    position_target_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // 设置速度
    position_target_msg.velocity.x = velocity.x();
    position_target_msg.velocity.y = velocity.y();
    position_target_msg.velocity.z = velocity.z();

    // 设置目标航向角
    position_target_msg.yaw = calculateYaw(current_pos, target_pos);

    // 发布指令
    setpoint_pub_.publish(position_target_msg);
}

// 发布姿态指令
void ProportionalNavigationNode::publishAttitude(float roll, float pitch, float yaw, float thrust)
{
    mavros_msgs::AttitudeTarget attitude_msg;
    attitude_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_THRUST;

    // 转换姿态为四元数
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    attitude_msg.orientation.x = q.x();
    attitude_msg.orientation.y = q.y();
    attitude_msg.orientation.z = q.z();
    attitude_msg.orientation.w = q.w();

    // 设置推力
    attitude_msg.thrust = thrust;
    attitude_msg.body_rate.x = 0.0;
    attitude_msg.body_rate.y = 0.0;
    attitude_msg.body_rate.z = 0.0;

    // // 发布消息
    attitude_pub_.publish(attitude_msg);
}

void ProportionalNavigationNode::computeStickControl()
{
    if (boundingbox_updated_ && boundingbox_.probability > 0.2)
    {

        double frame_center_x = (boundingbox_.xmin + boundingbox_.xmax) / 2.0;
        double frame_center_y = (boundingbox_.ymin + boundingbox_.ymax) / 2.0;

        // Calculate control signals
        double dx = target_center_x_ - frame_center_x;
        double dy = target_center_y_ - frame_center_y;

        // TODO From topics
        double rows = 480;
        double cols = 640;

        // 计算误差
        double error = std::sqrt(dx * dx + dy * dy);
        double alpha = alpha_min_ + (alpha_max_ - alpha_min_) * std::min(error / max_error_, 1.0); // 计算自适应 α
        std::cout << "dx: " << dx << "dy: " << dy << "error: " << error << std::endl;

        // Control calculations (simple proportional)
        double pitch = Kp_pitch_roll_ * dy / rows;
        double roll = Kp_pitch_roll_ * dx / cols;
        double yaw = Kp_yaw_ * dx / (cols / 5.0);
        // double thrust = 1.0 - Kp_thrust_ * std::abs(dy / rows);
        double thrust = (dy / (rows / 5.f));

        // Normalize the controls to the range [-1, 1] for joystick input
        double roll_input = roll;
        double pitch_input = pitch - 10.f / (180 / M_PI);
        double yaw_input = yaw;
        double thrust_input = thrust > 0.f ? 0.5f : thrust + 0.5f;
        // std::cout << "thrust: " << thrust  << "thrust_input: " << thrust_input << std::endl;

        // Ensure that the values are within [-1, 1] range
        roll_input = std::max(-1.0, std::min(1.0, roll_input / MAN_TILT_MAX));
        pitch_input = std::max(-1.0, std::min(1.0, -pitch_input / MAN_TILT_MAX));
        yaw_input = std::max(-1.0, std::min(1.0, -yaw_input));
        thrust_input = std::max(0.0, std::min(1.0, thrust_input));

        // **应用自适应低通滤波**
        roll_input = alpha * roll_input + (1 - alpha) * prev_roll_input_;
        pitch_input = alpha * pitch_input + (1 - alpha) * prev_pitch_input_;
        yaw_input = alpha * yaw_input + (1 - alpha) * prev_yaw_input_;
        thrust_input = alpha * thrust_input + (1 - alpha) * prev_thrust_input_;

        // **更新上一次的值**
        prev_roll_input_ = roll_input;
        prev_pitch_input_ = pitch_input;
        prev_yaw_input_ = yaw_input;
        prev_thrust_input_ = thrust_input;

        // std::cout << "alpha: " << alpha
        //     << " pitch_input: " << pitch_input
        //     << " roll_input: " << roll_input
        //     << " yaw_input: " << yaw_input
        //     << " thrust_input: " << thrust_input
        //     << std::endl;

        mavros_msgs::ManualControl manual_control_msg;
        manual_control_msg.header.stamp = ros::Time::now();

        // 将计算出的roll, pitch, yaw, thrust 转换为手动控制信号
        manual_control_msg.x = pitch_input * 1000.f;  // （pitch）
        manual_control_msg.y = roll_input * 1000.f;   // （roll）
        manual_control_msg.z = thrust_input * 1000.f; // （thrust）
        manual_control_msg.r = yaw_input * 1000.f;    // （yaw)

        manual_control_pub_.publish(manual_control_msg);
    }
    else
    {
        mavros_msgs::ManualControl manual_control_msg;
        manual_control_msg.header.stamp = ros::Time::now();

        // 将计算出的roll, pitch, yaw, thrust 转换为手动控制信号
        if (joy_updated_)
        {
            manual_control_msg.x = joy_.axes[4] * 1000.f;               // （pitch)
            manual_control_msg.y = -joy_.axes[3] * 1000.f;              // （roll)
            manual_control_msg.z = (joy_.axes[1] + 1.f) / 2.f * 1000.f; // (thrust）
            manual_control_msg.r = -joy_.axes[0] * 1000.f;              // （yaw)
            manual_control_pub_.publish(manual_control_msg);
        }
    }
}

// 计算姿态控制
mavros_msgs::ManualControl ProportionalNavigationNode::computeAttitudeControl()
{

    map_projection_project(gps_origin_.position.latitude, gps_origin_.position.longitude,
                           target_.latitude, target_.longitude, target_ned_.x(), target_ned_.y());

    target_velocity_.setZero();
    // 计算相对位置和速度
    Eigen::Vector3f relative_position = target_ned_ - drone_position_;
    Eigen::Vector3f relative_velocity = target_velocity_ - drone_velocity_;

    // std::cout << "target_ned_.x: " << target_ned_.x() << "target_ned_.y: " << target_ned_.y() <<"target_ned_.z: " << target_ned_.z() << std::endl;
    // std::cout << "drone_position_.x: " << drone_position_.x() << "drone_position_.y: " << drone_position_.y() <<"drone_position_.z: " << drone_position_.z() << std::endl;
    // std::cout << "target_velocity_.x: " << target_velocity_.x() << "target_velocity_.y: " << target_velocity_.y() <<"target_velocity_.z: " << target_velocity_.z() << std::endl;
    // std::cout << "drone_velocity_.x: " << drone_velocity_.x() << "drone_velocity_.y: " << drone_velocity_.y() <<"drone_velocity_.z: " << drone_velocity_.z() << std::endl;

    // 计算比例引导加速度
    Eigen::Vector3f cross_product = relative_velocity.cross(relative_position);
    Eigen::Vector3f acceleration = 2.0f * cross_product.cross(relative_position) / std::pow(relative_position.norm(), 2);

    // 转换为目标加速度（包括重力）
    Eigen::Vector3f target_acc = acceleration + Eigen::Vector3f(0, 0, GRAVITY);

    std::cout << "acceleration.x: " << acceleration.x() << "acceleration.y: " << acceleration.y() << "acceleration.z: " << acceleration.z() << std::endl;

    // 计算目标方向
    Eigen::Vector3f target_dir = target_acc.normalized();

    // 计算俯仰和滚转角度
    float pitch = std::asin(-target_dir.y());
    float roll = std::atan2(target_dir.x(), target_dir.z());
    pitch = std::clamp(pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    roll = std::clamp(roll, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);

    // 计算偏航角
    float Kp = 0.1f;
    float yaw = normalizeYaw(std::atan2(relative_position.y(), relative_position.x()));
    // 推力设置
    float thrust = std::min(1.0f, target_acc.norm() / GRAVITY);
    float yaw_err = normalizeYaw(yaw - drone_rpy_(2));
    std::cout << "yaw: " << yaw << "drone_rpy_(2): " << drone_rpy_(2) << "yaw_err: " << yaw_err << std::endl;
    mavros_msgs::ManualControl manual_control_msg;
    manual_control_msg.header.stamp = ros::Time::now();

    float dt = 0.1;
    // 将计算出的roll, pitch, yaw, thrust 转换为手动控制信号
    manual_control_msg.x = -pitch * dt * 1000.f;  // 左右摇杆（pitch）
    manual_control_msg.y = roll * dt * 1000.f;    // 前后摇杆（roll）
    manual_control_msg.z = thrust * 1000.f;       // 油门（thrust）
    manual_control_msg.r = Kp * yaw_err * 1000.f; // 偏航（yaw)

    std::cout << "pitch: " << pitch << "roll: " << roll << "yaw: " << yaw << "thrust: " << thrust << std::endl;

    return manual_control_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "proportional_navigation_node");

    ProportionalNavigationNode node;
    node.run();

    return 0;
}
