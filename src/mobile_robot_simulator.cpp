#include "ros/ros.h"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

MobileRobotSimulator::MobileRobotSimulator(ros::NodeHandle *nh)
{
    nh_ptr = nh;
    // get parameters
    get_params();
    actual_odom_pub = nh_ptr->advertise<nav_msgs::Odometry>(actual_odometry_topic,50); // odometry publisher
    odom_pub = nh_ptr->advertise<nav_msgs::Odometry>(odometry_topic,50); // odometry publisher
    vel_sub = nh_ptr->subscribe(velocity_topic,5,&MobileRobotSimulator::vel_callback,this); // velocity subscriber
    
    // initialize timers
    last_update = ros::Time::now();
    last_vel = last_update - ros::Duration(0.1);
    // initialize forst odom message
    update_odom_from_vel(geometry_msgs::Twist(), ros::Duration(0.1));
    odom.header.stamp = last_update;
    get_tf_from_odom(odom);
    // Initialize tf from map to odom
    if (publish_map_transform)
    {
        init_pose_sub = nh_ptr->subscribe("/initialpose",5,&MobileRobotSimulator::init_pose_callback,this); // initial pose callback
        map_trans.frame_id_ = "/map";
        map_trans.stamp_ = last_update;
        map_trans.child_frame_id_ = "/odom";
        map_trans.setIdentity();
    }

    check_pose_safety = nh_ptr->serviceClient<ikh_ros_msgs::SetFloatList>(check_pose_safety_service);    

    ROS_INFO("Initialized mobile robot simulator");
    
}

MobileRobotSimulator::~MobileRobotSimulator()
{
    if (is_running) stop();
}

void MobileRobotSimulator::get_params()
{
     nh_ptr->param<bool>("publish_map_transform", publish_map_transform , false);
     nh_ptr->param<double>("publish_rate", publish_rate, 10.0);
     nh_ptr->param<std::string>("velocity_topic", velocity_topic, "/cmd_vel");
     nh_ptr->param<std::string>("odometry_topic", odometry_topic, "/odom");
     nh_ptr->param<std::string>("actual_odometry_topic", actual_odometry_topic, "/actual_odom");
     nh_ptr->param<double>("drift_longitudinal_weight", drift_long, 0.1);
     nh_ptr->param<double>("drift_lateral_weight", drift_lat, 0.01);
     nh_ptr->param<double>("drift_angular_weight", drift_ang, 0.08);
     nh_ptr->param<bool>("use_drift_noise", use_drift_noise, true);
     nh_ptr->param<double>("noise_mean", mean_, 0.2);
     nh_ptr->param<double>("noise_dev", stddev_, 1);
     nh_ptr->param<double>("twist_pub_timeout", twist_pub_timeout, 1.0); //sec
     nh_ptr->param<bool>("pub_actual_odom_only_on_safe_areas", pub_only_safe, true); //sec
     nh_ptr->param<std::string>("check_pose_safety_service", check_pose_safety_service, "check_pose_safety");
}


void MobileRobotSimulator::start()
{
    loop_timer = nh_ptr->createTimer(ros::Duration(1.0/publish_rate),&MobileRobotSimulator::update_loop, this);
    loop_timer.start(); // should not be necessary
    is_running = true;
    ROS_INFO("Started mobile robot simulator update loop, listening on cmd_vel topic");
}

void MobileRobotSimulator::stop()
{
    loop_timer.stop();
    is_running = false;
    ROS_INFO("Stopped mobile robot simulator");
}

void MobileRobotSimulator::update_loop(const ros::TimerEvent& event)
{
    last_update = event.current_real;
    // If we didn't receive a message, send the old odometry info with a new timestamp
    if (!message_received)
    {
        odom.header.stamp = last_update;
        odom_trans.stamp_ = last_update;
    }

    if ((last_update-last_vel).toSec()>twist_pub_timeout)
    {
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.angular.z = 0.0;
        drift_dx = 0.0;
        drift_dy = 0.0;
    }

    // create drift odom
    odom_drift = odom;
    odom_drift.pose.pose.position.x = odom_drift.pose.pose.position.x + drift_x;
    odom_drift.pose.pose.position.y = odom_drift.pose.pose.position.y + drift_y;
    odom_drift.twist.twist.linear.x = odom_drift.twist.twist.linear.x + drift_dx;
    odom_drift.twist.twist.linear.y = odom_drift.twist.twist.linear.y + drift_dy;


    // create actual odom
    odom_actual = odom;
    odom_actual.header.frame_id = "map";

    // publish odometry and tf
    if (pub_only_safe)
    {   
        checkPoseSafetySrv.request.data.clear();
        checkPoseSafetySrv.request.data.push_back(odom_actual.pose.pose.position.x);
        checkPoseSafetySrv.request.data.push_back(odom_actual.pose.pose.position.y);

        if (check_pose_safety.call(checkPoseSafetySrv))
        {
            if (checkPoseSafetySrv.response.success){
                if (!checkPoseSafetySrv.response.result.front())
                {
                    // std::cerr << "Area is UNSAFE" << '\n';
                    safe_area = 0;
                }
                else{
                    // std::cerr << "Area is SAFE" << '\n';
                    safe_area = 1;
                }
            }
            else{
                std::cerr << "check_pose_safety service responded with an error" << '\n';
            }
        }
        else{
            std::cerr << "Could not call check_pose_safety service" << '\n';
        }
    }

    if (safe_area)
    {
        actual_odom_pub.publish(odom_actual);
    }
    
    odom_pub.publish(odom_drift);
    get_tf_from_odom(odom_drift);
    tf_broadcaster.sendTransform(odom_trans); // odom -> base_link
    message_received = false;
    // should we publish the map transform?
    if (!publish_map_transform) return;
    map_trans.stamp_ = last_update;
    tf_broadcaster.sendTransform(map_trans); // map -> odom
}

void MobileRobotSimulator::update_odom_from_vel(geometry_msgs::Twist vel, ros::Duration time_diff)
{
    ROS_DEBUG_STREAM("Velocity - x: " << vel.linear.x << " y: " << vel.linear.y << " th: " << vel.angular.z);
    //compute odometry in a typical way given the velocities of the robot
    double delta_x = (vel.linear.x * cos(th) - vel.linear.y * sin(th)) * time_diff.toSec();
    double delta_y = (vel.linear.x * sin(th) + vel.linear.y * cos(th)) * time_diff.toSec();
    double delta_th = vel.angular.z * time_diff.toSec();
    ROS_DEBUG_STREAM("Delta - x: " << delta_x << " y: " << delta_y << " th: " << delta_th);

    //update drift
    if(use_drift_noise)
    {
        std::random_device rd; 
        std::mt19937 generator(rd()); 
        std::normal_distribution<double> distr(mean_, stddev_);
        uncert_x = distr(generator);
        uncert_y = distr(generator);
    }

    double tmp_drift_x = drift_long*delta_x + drift_lat*delta_y + drift_ang*1.8*delta_th*uncert_x;
    double tmp_drift_y = drift_long*delta_y + drift_lat*delta_x + drift_ang*1.8*delta_th*uncert_y;
    drift_x = drift_x + tmp_drift_x;
    drift_y = drift_y + tmp_drift_y;
    drift_dx = tmp_drift_x/time_diff.toSec();
    drift_dy = tmp_drift_y/time_diff.toSec();
    
    // update odometry
    odom.header.stamp = measure_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x += delta_x;
    odom.pose.pose.position.y += delta_y;
    // generate quaternion based on current yaw
    th += delta_th;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    // set velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist = vel;
    ROS_DEBUG_STREAM("Odometry - x: " << odom.pose.pose.position.x << " y: " << odom.pose.pose.position.y << " th: " << th);
}

void MobileRobotSimulator::get_tf_from_odom(nav_msgs::Odometry odom)
{
    geometry_msgs::TransformStamped odom_tmp;
    // copy from odmoetry message
    odom_tmp.header = odom.header;
    odom_tmp.child_frame_id = odom.child_frame_id;
    odom_tmp.transform.translation.x = odom.pose.pose.position.x;
    odom_tmp.transform.translation.y = odom.pose.pose.position.y;
    odom_tmp.transform.translation.z = 0.0;
    odom_tmp.transform.rotation = odom.pose.pose.orientation;
    // convert and update
    tf::transformStampedMsgToTF(odom_tmp, odom_trans);
}

void MobileRobotSimulator::vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_DEBUG("Received message on cmd_vel");
    measure_time = ros::Time::now();
    ros::Duration dt = measure_time - last_vel;
    last_vel = measure_time;
    if (dt >= ros::Duration(0.5)) dt = ros::Duration(0.1);
    message_received = true;
    geometry_msgs::Twist vel = *msg;
    update_odom_from_vel(vel,dt);
}

void MobileRobotSimulator::init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (msg->header.frame_id != "map") {
        ROS_ERROR("Initial pose not specified in map frame, ignoring");
        return;
    }
    ROS_INFO("Received pose estimate of mobile base");
    
    // msg is map -> base_link
    tf::StampedTransform msg_t;
    msg_t.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    msg_t.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
    ROS_DEBUG_STREAM("map -> base_link - x: " << msg_t.getOrigin().getX() << " y: " << msg_t.getOrigin().getY());
    // get odom -> base_link
    ROS_DEBUG_STREAM("odom -> base_link - x: " << odom_trans.getOrigin().getX() << " y: " << odom_trans.getOrigin().getY());
    // calculate map -> odom and save as stamped
    tf::StampedTransform map_t = tf::StampedTransform(msg_t * odom_trans.inverse(), msg->header.stamp, "map", "odom");
    ROS_DEBUG_STREAM("map -> odom - x: " << map_t.getOrigin().getX() << " y: " << map_t.getOrigin().getY());
    map_trans = map_t;    
}



