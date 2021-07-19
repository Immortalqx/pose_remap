#include "Remap.h"
#include <gflags/gflags.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#define PI 3.1415926535

DEFINE_string(input_topic_type,
              "Odometry", "Available types: PoseStamped, PoseWithCovarianceStamped, Odometry");
DEFINE_string(input_topic_name,
              "/vio_pose", "VIO pose output. In flu coordinate");

DEFINE_string(output_topic_name,
              "/mavros/vision_pose/pose", "mavros VIO pose interface. In flu coordinate");
DEFINE_string(output_frame_id,
              "odom", "the frame id of the output pose");
DEFINE_bool(output_with_cov,
            false, "whether output with cov");
DEFINE_double(output_hz,
              50, "the rate of output to fcu");

DEFINE_double(T_map_global_tx,
              0., "translation of global frame to map(flu)");
DEFINE_double(T_map_global_ty,
              0., "translation of global frame to map(flu)");
DEFINE_double(T_map_global_tz,
              0., "translation of global frame to map(flu)");
DEFINE_double(T_map_global_r,
              0., "rotation of global frame to map(flu), rpy format");
DEFINE_double(T_map_global_p,
              0., "rotation of global frame to map(flu), rpy format");
DEFINE_double(T_map_global_y,
              0., "rotation of global frame to map(flu), rpy format");

DEFINE_double(R_imu1_imu0_r,
              0., "rotation of imu0 frame to imu1, rpy format");
DEFINE_double(R_imu1_imu0_p,
              0., "rotation of imu0 frame to imu1, rpy format");
DEFINE_double(R_imu1_imu0_y,
              0., "rotation of imu0 frame to imu1, rpy format");

DEFINE_double(R_body_imu1_r,
              0., "rotation of imu1 frame to body(flu), rpy format");
DEFINE_double(R_body_imu1_p,
              0., "rotation of imu1 frame to body(flu), rpy format");
DEFINE_double(R_body_imu1_y,
              0., "rotation of imu1 frame to body(flu), rpy format");

int T_map_global_type = 3;
std::map<std::string, int> enum_T_map_global =
        {{"T_map_global_r", 1},
         {"T_map_global_p", 2},
         {"T_map_global_y", 3}
        };

int R_imu1_imu0_type = 2;
std::map<std::string, int> enum_R_imu1_imu0 =
        {{"R_imu1_imu0_r", 1},
         {"R_imu1_imu0_p", 2},
         {"R_imu1_imu0_y", 3}
        };

int R_body_imu1_type = 1;
std::map<std::string, int> enum_R_body_imu1 =
        {{"R_body_imu1_r", 1},
         {"R_body_imu1_p", 2},
         {"R_body_imu1_y", 3}
        };

Eigen::AngleAxisd TMG_rollAngle;
Eigen::AngleAxisd TMG_pitchAngle;
Eigen::AngleAxisd TMG_yawAngle;

Eigen::AngleAxisd RII_rollAngle;
Eigen::AngleAxisd RII_pitchAngle;
Eigen::AngleAxisd RII_yawAngle;

Eigen::AngleAxisd RBI_rollAngle;
Eigen::AngleAxisd RBI_pitchAngle;
Eigen::AngleAxisd RBI_yawAngle;

geometry_msgs::PoseWithCovarianceStamped to_fcu_pose_cov;

Eigen::Isometry3d T_map_global;
Eigen::Quaterniond R_imu1_imu0;
Eigen::Quaterniond R_body_imu1;

std::string output_frame_id;

bool wait = true;

Remap::Remap()
= default;

void Remap::run(int argc, char **argv)
{

    ros::init(argc, argv, "remap_pose_node");
    ros::NodeHandle nh("~");

    google::ParseCommandLineFlags(&argc, &argv, true);

    // Transform initial
    transform_init();

    // ROS Publisher and Subscriber
    ros::Publisher pose_pub = make_publisher(nh);

    ros::Subscriber data_sub = make_subscriber(nh);

    //ROS ddynamic reconfigure
    ddynamic_register();

    ros::Rate wait_rate(10);
    while (ros::ok() && wait)
    {
        std::cout << "\033[33m" << "Waiting message from: " << FLAGS_input_topic_name << "\033[0m" << std::endl;

        ros::spinOnce();
        wait_rate.sleep();
    }

    std::cout << "\n\n"
              << "\033[34m" << "Start Pose Remap Node!" << "\033[0m" << std::endl
              << "\n\n";
    // spin
    ros::Rate publish_rate(FLAGS_output_hz);
    while (ros::ok())
    {
        if (FLAGS_output_with_cov)
        {
            pose_pub.publish(to_fcu_pose_cov);
        }
        else
        {
            geometry_msgs::PoseStamped to_fcu_pose;
            to_fcu_pose.header = to_fcu_pose_cov.header;
            to_fcu_pose.pose = to_fcu_pose_cov.pose.pose;
            pose_pub.publish(to_fcu_pose);
        }
        ros::spinOnce();
        publish_rate.sleep();
    }
}

void Remap::transform_init()
{
    // Transform initial
    T_map_global = Eigen::Isometry3d::Identity();

    TMG_rollAngle = Eigen::AngleAxisd(FLAGS_T_map_global_r, Eigen::Vector3d::UnitX());
    TMG_pitchAngle = Eigen::AngleAxisd(FLAGS_T_map_global_p, Eigen::Vector3d::UnitY());
    TMG_yawAngle = Eigen::AngleAxisd(FLAGS_T_map_global_y, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = TMG_yawAngle * TMG_pitchAngle * TMG_rollAngle;

    T_map_global.prerotate(q);
    T_map_global.pretranslate(Eigen::Vector3d(FLAGS_T_map_global_tx,
                                              FLAGS_T_map_global_ty,
                                              FLAGS_T_map_global_tz));

    RII_rollAngle = Eigen::AngleAxisd(FLAGS_R_imu1_imu0_r, Eigen::Vector3d::UnitX());
    RII_pitchAngle = Eigen::AngleAxisd(FLAGS_R_imu1_imu0_p, Eigen::Vector3d::UnitY());
    RII_yawAngle = Eigen::AngleAxisd(FLAGS_R_imu1_imu0_y, Eigen::Vector3d::UnitZ());
    R_imu1_imu0 = RII_yawAngle * RII_pitchAngle * RII_rollAngle;

    RBI_rollAngle = Eigen::AngleAxisd(FLAGS_R_body_imu1_r, Eigen::Vector3d::UnitX());
    RBI_pitchAngle = Eigen::AngleAxisd(FLAGS_R_body_imu1_p, Eigen::Vector3d::UnitY());
    RBI_yawAngle = Eigen::AngleAxisd(FLAGS_R_body_imu1_y, Eigen::Vector3d::UnitZ());
    R_body_imu1 = RBI_yawAngle * RBI_pitchAngle * RBI_rollAngle;

    output_frame_id = FLAGS_output_frame_id;
}

ros::Publisher Remap::make_publisher(ros::NodeHandle &nh)
{
    ros::Publisher pub;
    if (FLAGS_output_with_cov)
    {
        pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(FLAGS_output_topic_name, 1);
    }
    else
    {
        pub = nh.advertise<geometry_msgs::PoseStamped>(FLAGS_output_topic_name, 1);
    }
    return pub;
}

ros::Subscriber Remap::make_subscriber(ros::NodeHandle &nh)
{
    ros::Subscriber sub;
    if (FLAGS_input_topic_type == "PoseStamped")
    {
        sub = nh.subscribe<geometry_msgs::PoseStamped>(FLAGS_input_topic_name, 1, vio_pose_cb);
    }
    else if (FLAGS_input_topic_type == "PoseWithCovarianceStamped")
    {
        sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(FLAGS_input_topic_name, 1,
                                                                     vio_pose_cov_cb);
    }
    else if (FLAGS_input_topic_type == "Odometry")
    {
        sub = nh.subscribe<nav_msgs::Odometry>(FLAGS_input_topic_name, 1, vio_odom_cb);
    }
    return sub;
}

void Remap::ddynamic_register()
{
    ddynamic_reconfigure::DDynamicReconfigure ddr;
    ddr.registerEnumVariable<int>("TMG_type", 3,
                                  boost::bind(type_T_map_global_cb, _1),
                                  "TMG_type", enum_T_map_global);
    ddr.registerVariable<int>("T_map_global", int(round(FLAGS_T_map_global_y * 4 / PI)),
                              boost::bind(param_T_map_global_cb, _1),
                              "T_map_global", -4, 4);

    ddr.registerEnumVariable<int>("RII_type", 2,
                                  boost::bind(type_R_imu1_imu0_cb, _1),
                                  "RII_type", enum_R_imu1_imu0);
    ddr.registerVariable<int>("R_imu1_imu0", int(round(FLAGS_R_imu1_imu0_p * 4 / PI)),
                              boost::bind(param_R_imu1_imu0_cb, _1),
                              "T_map_global", -4, 4);

    ddr.registerEnumVariable<int>("RBI_type", 1,
                                  boost::bind(type_R_body_imu1_cb, _1),
                                  "RBI_type", enum_R_body_imu1);
    ddr.registerVariable<int>("R_body_imu1", int(round(FLAGS_R_body_imu1_r * 4 / PI)),
                              boost::bind(param_R_body_imu1_cb, _1),
                              "T_map_global", -4, 4);
    ddr.publishServicesTopics();
}

void Remap::vio_odom_cb(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &msg)
{
    Eigen::Isometry3d T_global_imu0 = Eigen::Isometry3d::Identity();
    tf2::fromMsg(msg->pose.pose, T_global_imu0);

    Eigen::Isometry3d pose_in_global_frame = T_global_imu0 * (R_imu1_imu0.inverse() * R_body_imu1.inverse());

    to_fcu_pose_cov.header.stamp = ros::Time::now();
    to_fcu_pose_cov.header.frame_id = output_frame_id;
    to_fcu_pose_cov.pose.pose = tf2::toMsg(T_map_global * pose_in_global_frame);
}

void Remap::vio_pose_cb(const geometry_msgs::PoseStamped_<std::allocator<void>>::ConstPtr &msg)
{
    Eigen::Isometry3d T_global_imu0 = Eigen::Isometry3d::Identity();
    tf2::fromMsg(msg->pose, T_global_imu0);

    Eigen::Isometry3d pose_in_global_frame = T_global_imu0 * (R_imu1_imu0.inverse() * R_body_imu1.inverse());

    to_fcu_pose_cov.header.stamp = ros::Time::now();
    to_fcu_pose_cov.header.frame_id = output_frame_id;
    to_fcu_pose_cov.pose.pose = tf2::toMsg(T_map_global * pose_in_global_frame);

    wait = false;
}

void Remap::vio_pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped_<std::allocator<void>>::ConstPtr &msg)
{
    Eigen::Isometry3d T_global_imu0 = Eigen::Isometry3d::Identity();
    tf2::fromMsg(msg->pose.pose, T_global_imu0);

    Eigen::Isometry3d pose_in_global_frame = T_global_imu0 * (R_imu1_imu0.inverse() * R_body_imu1.inverse());

    to_fcu_pose_cov.header.stamp = ros::Time::now();
    to_fcu_pose_cov.header.frame_id = output_frame_id;
    to_fcu_pose_cov.pose.pose = tf2::toMsg(T_map_global * pose_in_global_frame);

    wait = false;
}

void Remap::type_T_map_global_cb(int type)
{
    T_map_global_type = type;
}

void Remap::param_T_map_global_cb(int value)
{
    T_map_global = Eigen::Isometry3d::Identity();

    if (T_map_global_type == 1)
        TMG_rollAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitX());
    if (T_map_global_type == 2)
        TMG_pitchAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitY());
    if (T_map_global_type == 3)
        TMG_yawAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = TMG_yawAngle * TMG_pitchAngle * TMG_rollAngle;

    T_map_global.prerotate(q);
    T_map_global.pretranslate(Eigen::Vector3d(FLAGS_T_map_global_tx,
                                              FLAGS_T_map_global_ty,
                                              FLAGS_T_map_global_tz));
}

void Remap::type_R_imu1_imu0_cb(int type)
{
    R_imu1_imu0_type = type;
}

void Remap::param_R_imu1_imu0_cb(int value)
{
    if (R_imu1_imu0_type == 1)
        RII_rollAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitX());
    if (R_imu1_imu0_type == 2)
        RII_pitchAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitY());
    if (R_imu1_imu0_type == 3)
        RII_yawAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitZ());

    R_imu1_imu0 = RII_yawAngle * RII_pitchAngle * RII_rollAngle;
}

void Remap::type_R_body_imu1_cb(int type)
{
    R_body_imu1_type = type;
}

void Remap::param_R_body_imu1_cb(int value)
{
    if (R_body_imu1_type == 1)
        RBI_rollAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitX());
    if (R_body_imu1_type == 2)
        RBI_pitchAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitY());
    if (R_body_imu1_type == 3)
        RBI_yawAngle = Eigen::AngleAxisd(PI * value / 4, Eigen::Vector3d::UnitZ());

    R_body_imu1 = RBI_yawAngle * RBI_pitchAngle * RBI_rollAngle;
}

