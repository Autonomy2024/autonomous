#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <math.h>
#include <Eigen/Dense>


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
    double lon, float& x, float& y)
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
    if (abs(c) > 0) {
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

    if (scale_factor < 1.f) {
        if (max_xy_norm < accuracy && xynorm < accuracy) {
            target.head<2>() = target.head<2>() * scale_factor;

        } else {
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

    if (scale_factor < 1.f) {
        if (max_z_norm < accuracy && znorm < accuracy) {
            target(2) *= scale_factor;

        } else {
            target *= scale_factor;
        }
    }
}


class ProportionalNavigationNode
{
public:
    ProportionalNavigationNode()
    {
        // 初始化发布器和订阅器
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("iris_0/mavros/setpoint_velocity/cmd_vel", 10);
        state_sub_ = nh_.subscribe("iris_0/mavros/state", 10, &ProportionalNavigationNode::stateCallback, this);
        position_sub_ = nh_.subscribe("iris_0/mavros/local_position/pose", 10, &ProportionalNavigationNode::positionCallback, this);

        target_sub_ = nh_.subscribe("rover_1/mavros/global_position/global", 10, &ProportionalNavigationNode::targetCallback, this);
        gps_origin_sub_ = nh_.subscribe("iris_0/mavros/global_position/gp_origin", 10, &ProportionalNavigationNode::gpsOriginCallback, this);

        // 服务客户端，用于设置模式和解锁
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("iris_0/mavros/set_mode");
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("iris_0/mavros/cmd/arming");

    }

    void run()
    {
        ros::Rate rate(20); // 20 Hz 控制频率

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

        // 请求切换到 OFFBOARD 模式并解锁
        if (!setOffboardAndArm())
        {
            ROS_ERROR("Failed to set OFFBOARD mode or arm the drone!");
            return;
        }

        while (ros::ok())
        {
            // if (!current_state.armed) {
            //     global_origin_(0) = 
            // }
        
            // 检查当前状态
            if (current_state_.mode != "OFFBOARD" || !current_state_.armed)
            {
                ROS_WARN_THROTTLE(1, "OFFBOARD mode or arming lost. Trying to re-engage...");
                if (!setOffboardAndArm())
                {
                    ROS_ERROR("Re-engagement failed. Exiting...");
                    return;
                }
            }

            // 计算并发布速度指令
            geometry_msgs::TwistStamped cmd_vel = computeProportionalNavigation();
            velocity_pub_.publish(cmd_vel);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher velocity_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber position_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber gps_origin_sub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_position_;
    sensor_msgs::NavSatFix target_;
    geographic_msgs::GeoPointStamped gps_origin_;

    Eigen::Vector3d global_origin_;

    void stateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state_ = *msg;
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_position_ = *msg;
    }

    void targetCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        target_ = *msg;

    }

    void gpsOriginCallback(const geographic_msgs::GeoPointStamped::ConstPtr &msg)
    {
        gps_origin_ = *msg;

    }

    bool setOffboardAndArm()
    {
        // 设置 OFFBOARD 模式
        mavros_msgs::SetMode set_mode_req;
        set_mode_req.request.custom_mode = "OFFBOARD";

        if (set_mode_client_.call(set_mode_req) && set_mode_req.response.mode_sent)
        {
            ROS_INFO("OFFBOARD mode enabled");
        }
        else
        {
            ROS_ERROR("Failed to set OFFBOARD mode");
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

    geometry_msgs::TwistStamped computeProportionalNavigation()
    {
        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();

        // 无人机当前位置
        double ux = current_position_.pose.position.x;
        double uy = current_position_.pose.position.y;
        double uz = current_position_.pose.position.z;

        Eigen::Vector3f target_ned{0,0,0.f};
        ROS_INFO("gps_origin_.position.latitude: %.7f, gps_origin_.position.longitude: %.7f",gps_origin_.position.latitude, gps_origin_.position.longitude);
        ROS_INFO("target_.latitude: %.7f,  target_.longitude: %.7f",target_.latitude,  target_.longitude);

        map_projection_project(gps_origin_.position.latitude, gps_origin_.position.longitude, target_.latitude, target_.longitude, target_ned.x(), target_ned.y());
        ROS_INFO("target_ned.x: %.2f, target_ned.y: %.2f, target_ned.z: %.2f", target_ned.x(), target_ned.y(), target_ned.z());
        // 目标点位置
        double tx = target_ned.x();
        double ty = target_ned.y();
        double tz = target_ned.z();

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
        Eigen::Vector3f target_pos(target_ned.x(), target_ned.y(), target_ned.z());
        Eigen::Vector3f pos_to_target = (target_pos - cur_pos).normalized();
        Eigen::Vector3f vel_sp_constrained = pos_to_target * (max_xy_vel * max_xy_vel + max_z_vel * max_z_vel);
        clampToXYNorm(vel_sp_constrained, max_xy_vel, 0.1f);
        clampToZNorm(vel_sp_constrained, max_z_vel, 0.1f);

        float vel_xy_norm = sqrtf(cmd_vel.twist.linear.x * cmd_vel.twist.linear.x + cmd_vel.twist.linear.y * cmd_vel.twist.linear.y);
        float vel_z_norm = fabsf(cmd_vel.twist.linear.z);
    
        if (vel_xy_norm > vel_sp_constrained.head<2>().norm()) {
            cmd_vel.twist.linear.x *= vel_sp_constrained.head<2>().norm() / vel_xy_norm;
            cmd_vel.twist.linear.y *= vel_sp_constrained.head<2>().norm() / vel_xy_norm;
        }

        if (vel_z_norm > fabsf(vel_sp_constrained.z())) {
            cmd_vel.twist.linear.z *= fabsf(vel_sp_constrained.z()) / vel_z_norm;
        }

        return cmd_vel;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "proportional_navigation_node");

    ProportionalNavigationNode node;
    node.run();

    return 0;
}
