#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "ikh_ros_msgs/SetFloatList.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <functional>
#include <random>
#include <memory>

#ifndef MOBILE_ROBOT_SIMULATOR
#define MOBILE_ROBOT_SIMULATOR

class MobileRobotSimulator {

public:

    MobileRobotSimulator(ros::NodeHandle *nh); // default constructor
    ~MobileRobotSimulator(); // default destructor
    
    /*! start the simulation loop */
    void start(); // 
    
    /*! stop everything */
    void stop();
    
    bool publish_map_transform; // whether or not to publish the map transform


private:

    /*! gets parameters from the parameter server */
    void get_params();

    /*! main update loop */
    void update_loop(const ros::TimerEvent& event);
    
    /*! update the odometry info based on velocity and duration */
    void update_odom_from_vel(geometry_msgs::Twist vel, ros::Duration time_diff);

    /*! generate transform from odom */
    void get_tf_from_odom(nav_msgs::Odometry odom);

    /*! callback function for velocity */
    void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

    /*! initial pose callback function */
    void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


    double publish_rate;

    nav_msgs::Odometry odom; // odometry message
    tf::StampedTransform odom_trans; // odometry transform
    tf::StampedTransform map_trans; // transformation from odom to map

    // Drift-related stuff
    nav_msgs::Odometry odom_drift; // odometry with drift noise message
    nav_msgs::Odometry odom_actual; // odometry actual message
    double drift_x = 0.0;
    double drift_y = 0.0;
    double drift_dx = 0.0;
    double drift_dy = 0.0;
    double mean_ = 0.2;
    double stddev_ = 1;
    double drift_long;
    double drift_lat;
    double drift_ang;
    bool use_drift_noise;
    double uncert_x = 1;
    double uncert_y = 1;
    double twist_pub_timeout = 1.0;

    ros::Time last_vel; // last incoming velocity command
    ros::Time last_update; // last time the odom was published
    ros::Time measure_time; // this incoming velocity command
    bool message_received = false;
    ros::NodeHandle * nh_ptr;
    
    bool is_running;
    
    // ROS interfaces
    ros::Publisher odom_pub;
    ros::Publisher actual_odom_pub;
    ros::Publisher rtk_odom_pub;
    ros::Subscriber vel_sub;
    ros::Subscriber init_pose_sub;    
    tf::TransformBroadcaster tf_broadcaster; 

    //check pose safety
    ros::ServiceClient check_pose_safety;
    ikh_ros_msgs::SetFloatList checkPoseSafetySrv;
    std::string check_pose_safety_service;
    bool safe_area = 1;
    bool pub_only_safe;

    //Topics
    std::string velocity_topic;
    std::string odometry_topic;
    std::string actual_odometry_topic;
    std::string rtk_odometry_topic;
    
    ros::Timer loop_timer; // timer for the update loop
    
    double th = 0.0; // current pose (only need yaw, rest is calculated)

    

}; // end class

#endif
