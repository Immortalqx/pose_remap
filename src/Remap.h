#ifndef POSE_REMAP_REMAP_H
#define POSE_REMAP_REMAP_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class Remap
{
public:
    Remap();

    static void run(int argc, char **argv);

private:
    static void transform_init();

    static ros::Publisher make_publisher(ros::NodeHandle &nh);

    static ros::Subscriber make_subscriber(ros::NodeHandle &nh);

    static void ddynamic_register();

    static void vio_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);

    static void vio_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    static void vio_pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    static void type_T_map_global_cb(int type);

    static void param_T_map_global_cb(int value);

    static void type_R_imu1_imu0_cb(int type);

    static void param_R_imu1_imu0_cb(int value);

    static void type_R_body_imu1_cb(int type);

    static void param_R_body_imu1_cb(int value);
};


#endif //POSE_REMAP_REMAP_H
