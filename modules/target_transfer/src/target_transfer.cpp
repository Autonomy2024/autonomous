#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/FollowTarget.h>
#include <nav_msgs/Odometry.h>
#include <string>

class TargetTransfer {
public:
    TargetTransfer() {
        // 初始化订阅者和发布者
        std::string source;
        std::string dest;
	      nh_.param<std::string>("source", source, "/rover_1");
        nh_.param<std::string>("dest", dest, "/iris_0");
        global_pos_sub_ = nh_.subscribe(source+"/mavros/global_position/global", 10, &TargetTransfer::globalPosCallback, this);
        gps_vel_sub_ = nh_.subscribe(source+"/mavros/global_position/raw/gps_vel", 10, &TargetTransfer::gpsVelCallback, this);
        target_vel_sub_ = nh_.subscribe(source+"/mavros/local_position/odom", 10, &TargetTransfer::TargetVelCallback, this);

        target_pub_ = nh_.advertise<mavros_msgs::FollowTarget>(dest+"/mavros/target_localization", 10);
        
    }

    // 处理全球定位回调
    void globalPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        global_position_ = *msg;
    }

    // 处理GPS速度回调
    void gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        gps_velocity_ = *msg;
    }
    
    // 处理GPS速度回调
    void TargetVelCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        target_velocity_ = *msg;

    }

    // 合并并发布目标位置
    void mergeAndPublish() {
        // 确保已经收到了全球定位和GPS速度数据
            mavros_msgs::FollowTarget target_msg;

            target_msg.latitude = global_position_.latitude;
            target_msg.longitude = global_position_.longitude;
            target_msg.altitude = global_position_.altitude;

            // target_msg.vx = gps_velocity_.twist.linear.x;
            // target_msg.vy = gps_velocity_.twist.linear.y;
            // target_msg.vz = gps_velocity_.twist.linear.z;

            target_msg.vx = target_velocity_.twist.twist.linear.x;
            target_msg.vy = target_velocity_.twist.twist.linear.y;
            target_msg.vz = target_velocity_.twist.twist.linear.z;

            // 发布合并后的目标数据
            target_pub_.publish(target_msg);
          // ROS_INFO("Published target global position: [%f, %f, %f]", target_msg.latitude, target_msg.longitude, target_msg.altitude);
          // ROS_INFO("Published target velocity: [%f, %f, %f]", target_msg.vx, target_msg.vy, target_msg.vz);

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber global_pos_sub_;
    ros::Subscriber gps_vel_sub_, target_vel_sub_;
    ros::Publisher target_pub_;

    sensor_msgs::NavSatFix global_position_; 
    geometry_msgs::TwistStamped gps_velocity_;
    nav_msgs::Odometry target_velocity_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_transfer");

    TargetTransfer target_transfer;

    ros::Rate loop_rate(10); // 设置循环频率为10Hz

    while (ros::ok()) {
        target_transfer.mergeAndPublish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
