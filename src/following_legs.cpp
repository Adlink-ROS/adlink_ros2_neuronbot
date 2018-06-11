// Copyright 2018 ADLINK Technology, Inc.
// Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_FATAL RCUTILS_LOG_FATAL
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_DEBUG RCUTILS_LOG_DEBUG

#include <iostream>
#include <string>
#include <math.h> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp" 

#include "adlink_msgs/msg/person_array.hpp"
#include "adlink_msgs/msg/person.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// for timer
using namespace std::chrono_literals;

// =============================
// ===== Class Declaration =====
// =============================
class FollowMe : public rclcpp::Node
{    
    private:
        double Ki_linear_window_, Ki_angular_window_, linear_bound_, angular_bound_;
        int target_id_;
        bool status_, debug_;
        double sum_error_r_, sum_error_th_, previous_time_;
        double following_distance_, dead_zone_radius_, dead_zone_theta_, search_radius_, search_theta_;
        double Kp_linear_, Kp_angular_, Ki_linear_, Ki_angular_, Ki_factor_linear_, Ki_factor_angular_;
        std::string base_frame_ ;

        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener tf2_;

        geometry_msgs::msg::Twist twist_;
        adlink_msgs::msg::PersonArray personArray_;
        
        rclcpp::Subscription<adlink_msgs::msg::PersonArray>::SharedPtr person_array_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
     
        void trackerCB(const adlink_msgs::msg::PersonArray::SharedPtr msg);
        int findTargetId(const adlink_msgs::msg::PersonArray &person_array);
        bool xyToPolar(const adlink_msgs::msg::PersonArray &person_array, const int &target_id, double &radius, double &theta);
        double constrain(const double &input, const double &limit_upper, const double &limit_lower);
        void sendCmd(const double &linear_x, const double &angular_z);
        void controlCB();

    public:
        FollowMe();
        ~FollowMe();

}; // end of class declaration


// ================================
// ===== Class Implementation =====
// ================================
FollowMe::FollowMe() : Node("following_legs"), tf2_(buffer_)
{
    // Init variables    
    target_id_ = -1;
    sum_error_r_ = 0;
    sum_error_th_ = 0;
    previous_time_ = 0; // in sec
    status_ = false; // initial status

    // Get parameters
    this->get_parameter_or("debug", debug_, true);
    this->get_parameter_or("Ki_linear_window", Ki_linear_window_, 0.95);
    this->get_parameter_or("Ki_angular_window", Ki_angular_window_, 0.95);
    this->get_parameter_or("linear_bound", linear_bound_, 0.17);
    this->get_parameter_or("angular_bound", angular_bound_, 1.2);
    this->get_parameter_or("following_distance", following_distance_, 0.9);
    this->get_parameter_or("dead_zone_radius", dead_zone_radius_, 0.1); // +/- m
    this->get_parameter_or("dead_zone_theta", dead_zone_theta_, 0.1); // +/- rad
    this->get_parameter_or("search_radius", search_radius_, 1.5); // +/- m
    this->get_parameter_or("search_theta", search_theta_, 0.4); // +/- rad
    this->get_parameter_or("Kp_linear", Kp_linear_, 0.45);
    this->get_parameter_or("Kp_angular", Kp_angular_, 1.0);
    this->get_parameter_or("Ki_linear", Ki_linear_, 0.0);
    this->get_parameter_or("Ki_angular", Ki_angular_, 0.0);
    this->get_parameter_or("Ki_factor_linear", Ki_factor_linear_, 0.95);
    this->get_parameter_or("Ki_factor_angular", Ki_factor_angular_, 0.95);
    this->get_parameter_or("base_frame", base_frame_, std::string("/base_footprint"));

    // Create: Publisher, Subscriber & Timer
    person_array_sub_ = this->create_subscription<adlink_msgs::msg::PersonArray>(
                      "/following/legs", std::bind(&FollowMe::trackerCB, this, std::placeholders::_1), rmw_qos_profile_sensor_data); //topic, QoS

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/following/cmd_vel", rmw_qos_profile_sensor_data); //topic, QoS

    timer_ = this->create_wall_timer(100ms, std::bind(&FollowMe::controlCB, this)); // 10Hz (100ms)

    // Init TF2
    //tf2_ = tf2_ros::TransformListener(buffer_);

    twist_ = geometry_msgs::msg::Twist();

    previous_time_ = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds()/10e9;
}


FollowMe::~FollowMe()
{
    // None
}


void FollowMe::trackerCB(const adlink_msgs::msg::PersonArray::SharedPtr msg)
{
    personArray_ = *msg;
    if(target_id_ == -1 && personArray_.people.size() > 0) 
        target_id_ = findTargetId(personArray_);
    
    if(target_id_ >= 0)
        status_ = true; // start following (for timer loop)
    else
        status_ = false; // stop following (for timer loop)
}


int FollowMe::findTargetId(const adlink_msgs::msg::PersonArray &person_array)
{
    for(int i=0; i<person_array.people.size(); i++)
    {
        double r, th;
        xyToPolar(person_array, i, r, th);
        if(r <= search_radius_ && th <= search_theta_ && th >= -search_theta_)
            return person_array.people[i].id;
    }
}


bool FollowMe::xyToPolar(const adlink_msgs::msg::PersonArray &person_array, const int &target_id, double &radius, double &theta)
{   
    try // transform Pose from /odom to /base_link
    {      
        geometry_msgs::msg::PoseStamped basePose, odomPose;
        odomPose.header = person_array.header;
        odomPose.pose = person_array.people[target_id].pose;         
        buffer_.transform(odomPose, basePose, base_frame_); // in, out, target_frame
        // to polar coordinate
        double target_x = basePose.pose.position.x;
        double target_y = basePose.pose.position.y;
        radius = sqrt( pow(target_x, 2) + pow(target_y, 2) );
        theta  = atan2(target_y,target_x);
    } 
    catch (tf2::TransformException &e) 
    {
        ROS_WARN("Failed to find latest transform! (%s)", e.what());
        radius = -1;
        theta  = -1;        
        return false;
    }
    
    return true;
}


double FollowMe::constrain(const double &input, const double &limit_upper, const double &limit_lower)
{
    if(input > limit_upper)
        return limit_upper;
    else if(input < limit_lower) 
        return limit_lower;
    else
        return input;
}


void FollowMe::sendCmd(const double &linear_x, const double &angular_z)
{
    twist_.linear.x = linear_x;
    twist_.angular.z = angular_z;
    twist_pub_->publish(twist_);
}


void FollowMe::controlCB()
{
    // Timer callback at 10 Hz
    if(!status_ || target_id_ < 0)
    {
        if(debug_) ROS_INFO("Stop following !");
        sendCmd(0.0, 0.0);
        previous_time_ = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds()/10e9; // reset time
        sum_error_r_ = 0;
        sum_error_th_ = 0;
        return;
    }

    // Calculate PI controller
    // check target existing & find target's radius & theta
    bool target_get = false;  
    double r = -1;
    double th;
    for(int i=0; i<= personArray_.people.size(); i++)
        if(personArray_.people[i].id == target_id_)
            if( xyToPolar(personArray_, i, r, th) )
                target_get = true;

    // check result
    if(!target_get || r < 0.0)
    {
        if(debug_) ROS_INFO("Target lost !");
        sendCmd(0.0, 0.0);
        previous_time_ = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds()/10e9; // reset time
        sum_error_r_ = 0;
        sum_error_th_ = 0;
        return;
    }
    
    // compute error values
    double error_r  = r - following_distance_;
    double error_th = th - 0.0;

    // compute dt
    double current_time = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds()/10e9;
    double dt = current_time - previous_time_;

    // calculate cmd
    double follow_speed_x = 0.0;
    double follow_speed_z = 0.0;
    if ( abs(error_r) >= dead_zone_radius_ )
    {
        sum_error_r_  = Ki_factor_linear_*Ki_linear_window_*sum_error_r_ + error_r*dt;
        follow_speed_x = Kp_linear_*error_r + Ki_linear_ *sum_error_r_;
        follow_speed_x = constrain(follow_speed_x, linear_bound_, -linear_bound_);
    }
    else
    {    
        follow_speed_x = 0.0;
        if(debug_) ROS_INFO("Linear Dead Zone!");
    }

    if( abs(error_th) >= dead_zone_theta_)
    {
        sum_error_th_ = Ki_factor_angular_*Ki_angular_window_*sum_error_th_ + error_th*dt;
        follow_speed_z = Kp_angular_*th + Ki_angular_*sum_error_th_;
        follow_speed_z = constrain(follow_speed_z, angular_bound_, -angular_bound_);
    }
    else
    {
        follow_speed_z = 0.0;
        if(debug_) ROS_INFO("Angular Dead Zone!");
    }

    // Pub vel cmd
    sendCmd(follow_speed_x, follow_speed_z);
    previous_time_ = current_time;
    if(debug_)
    {
        ROS_INFO("Target ID: %d, r: %f, th: %f", target_id_, r, th);
        ROS_INFO("PID linear: %f, angular: %f", follow_speed_x, follow_speed_z);
    }
}

// =========================
// ===== Main Function =====
// =========================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto fm = std::make_shared<FollowMe>();
    rclcpp::spin(fm);
    rclcpp::shutdown();
    return 0;
}








