#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <functional>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <mutex>

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

    ros::Time last_vel; // last incoming velocity command
    ros::Time last_update; // last time the odom was published
    ros::Time measure_time; // this incoming velocity command
    bool message_received = false;
    ros::NodeHandle * nh_ptr;
    
    bool is_running;
    
    // ROS interfaces
    ros::Publisher odom_pub;
    ros::Subscriber vel_sub;
    ros::Subscriber init_pose_sub;    
    tf::TransformBroadcaster tf_broadcaster; 

    // Localization Jump Extension
    ros::Subscriber trigger_jump_sub;
    std::string trigger_jump_topic;
    ros::Subscriber costmap_sub;
    std::string costmap_topic;
    nav_msgs::Odometry odom_jumped, odom_old;
    bool mode = 0;
    nav_msgs::OccupancyGrid costmap;
    // std::shared_ptr<Eigen::MatrixXf> map;
    void trigger_jump_callback(const std_msgs::Bool::ConstPtr& msg);
    void execute_jump();
    void get_occupied_pose();
    void get_costmap();
    int THRESHOLD = 90;
    std::mutex mtx; 
    // const std::lock_guard<std::mutex> lock(mtx);

    
    //Topics
    std::string velocity_topic;
    std::string odometry_topic;
    
    ros::Timer loop_timer; // timer for the update loop
    
    double th = 0.0; // current pose (only need yaw, rest is calculated)

    

}; // end class

#endif
