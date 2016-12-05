/*
 * Modified by Liyiying
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"

#include "std_msgs/Float64.h"

#include <iostream>
using namespace std;

namespace turtlebot_follower
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//* The turtlebot follower nodelet.
/**
 * The turtlebot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
class TurtlebotFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  TurtlebotFollower() : min_y_(0), max_y_(0.6),
                        min_x_(-0.5), max_x_(0.5),
                        max_z_(0.8), goal_z_(0.3),
                        z_scale_(1.0), x_scale_(5.0)
  {

  }

  ~TurtlebotFollower()
  {
    delete config_srv_;
  }

private:
  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */
  std_msgs::Float64 dist_msg;
  std_msgs::Float64 dist_msg1;
  std_msgs::Float64 dist_msg2;
  std_msgs::Float64 dist_msg3;
  std_msgs::Float64 dist_msg4;

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);

    distpub_ = private_nh.advertise<std_msgs::Float64> ("dist_object", 1);
    distpub1_ = private_nh.advertise<std_msgs::Float64> ("dist_object1", 1);//right-down
    distpub2_ = private_nh.advertise<std_msgs::Float64> ("dist_object2", 1);//right-up
    distpub3_ = private_nh.advertise<std_msgs::Float64> ("dist_object3", 1);//left-down
    distpub4_ = private_nh.advertise<std_msgs::Float64> ("dist_object4", 1);//left-up

///    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
    sub_= nh.subscribe<PointCloud>("depth/points", 1, &TurtlebotFollower::cloudcb, this);

    switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>::CallbackType f =
        boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
  }

  void reconfigure(turtlebot_follower::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /*!
   * @brief Callback for point clouds.
   * Callback for point clouds. Uses PCL to find the centroid
   * of the points in a box in the center of the point cloud.
   * Publishes cmd_vel messages with the goal from the cloud.
   * @param cloud The point cloud message.
   */
  void cloudcb(const PointCloud::ConstPtr&  cloud)
  {
    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    float z1= 1e6;
    float z2= 1e6;
    float z3= 1e6;
    float z4= 1e6;
    float x1 = 0.0;
    float y1 = 0.0;
    float x2 = 0.0;
    float y2 = 0.0;
    float x3 = 0.0;
    float y3 = 0.0;
    float x4 = 0.0;
    float y4 = 0.0;

    //Number of points observed
    unsigned int n = 0;
    unsigned int n1 = 0;
    unsigned int n2 = 0;
    unsigned int n3 = 0;
    unsigned int n4 = 0;
    //Iterate through all the points in the region and find the average of the position
    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
    {
      //First, ensure that the point's position is valid. This must be done in a seperate
      //if because we do not want to perform comparison on a nan value.
      if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
      {
        //Test to ensure the point is within the aceptable box.
        if (pt.y > min_y_ && pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_)
        {
          //Add the point to the totals
          x += pt.x;
          y += pt.y;
          z = std::min(z, pt.z);
          //z += pt.z;
          n++;
        }
        if (pt.y > min_y_ && pt.y < 0.3 && pt.x < 0 && pt.x > min_x_) //right-down
        {
          //Add the point to the totals
          x1 += pt.x;
          y1 += pt.y;
          z1 = std::min(z1, pt.z);
          //z1 += pt.z;
          n1++;
        }
       if (pt.y > 0.3 && pt.y < max_y_ && pt.x < 0 && pt.x > min_x_) //right-up
        {
          //Add the point to the totals
          x2 += pt.x;
          y2 += pt.y;
          z2 = std::min(z2, pt.z);
          //z2 += pt.z;
          n2++;
        }
        if (pt.y > min_y_ && pt.y < 0.3 && pt.x < max_x_ && pt.x > 0) //left-down
        {
          //Add the point to the totals
          x3 += pt.x;
          y3 += pt.y;
          z3 = std::min(z3, pt.z);
          //z3 += pt.z;
          n3++;
        }
        if (pt.y > 0.3 && pt.y < max_y_ && pt.x < max_x_ && pt.x > 0) //left-up
        {
          //Add the point to the totals
          x4 += pt.x;
          y4 += pt.y;
          z4 = std::min(z4, pt.z);
          //z4 += pt.z;
          n4++;
        }
      }
    }

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>100)
    {
      x /= n;
      y /= n;

      //z /= n;
      
      //z1 /= n1;
      //z2 /= n2;
      //z3 /= n3;
      //z4 /= n4;

      if(n1==0){z1=0;}
      if(n2==0){z2=0;}
      if(n3==0){z3=0;}
      if(n4==0){z4=0;}

      dist_msg.data = z;
      
      dist_msg1.data = z1;
      dist_msg2.data = z2;
      dist_msg3.data = z3;
      dist_msg4.data = z4;

      //ROS_INFO("dist_msg.data %f", dist_msg.data);
      cout << "dist_msg.data = " << dist_msg.data << endl ;
      cout << "x = " << x << endl;
      cout << "y = " << y << endl;
      cout << "n = " << n << endl;
      cout << "dist_msg1.data = " << dist_msg1.data << endl ;
      cout << "x1 = " << x1 << endl;
      cout << "y1 = " << y1 << endl;
      cout << "n1 = " << n1 << endl;
      cout << "dist_msg2.data = " << dist_msg2.data << endl ;
      cout << "x2 = " << x2 << endl;
      cout << "y2 = " << y2 << endl;
      cout << "n2 = " << n2 << endl;
      cout << "dist_msg3.data = " << dist_msg3.data << endl ;
      cout << "x3 = " << x3 << endl;
      cout << "y3 = " << y3 << endl;
      cout << "n3 = " << n3 << endl;
      cout << "dist_msg4.data = " << dist_msg4.data << endl ;
      cout << "x4 = " << x4 << endl;
      cout << "y4 = " << y4 << endl;
      cout << "n4 = " << n4 << endl;

      distpub_.publish (dist_msg);
      distpub1_.publish (dist_msg1);
      distpub2_.publish (dist_msg2);
      distpub3_.publish (dist_msg3);
      distpub4_.publish (dist_msg4);

/*      if(z > max_z_){
        ROS_DEBUG("No valid points detected, stopping the robot");
        if (enabled_)
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        return;
      }

      ROS_DEBUG("Centroid at %f %f %f with %d points", x, y, z, n);
      publishMarker(x, y, z);

      if (enabled_)
      {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = (z - goal_z_) * z_scale_;
        cmd->angular.z = -x * x_scale_;
        cmdpub_.publish(cmd);
      }*/
    }
    else
    {
      ROS_DEBUG("No points detected, stopping the robot");
      publishMarker(x, y, z);

/*      if (enabled_)
      {
        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }*/
    }

    publishBbox();
  }

  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
    {
      ROS_INFO("Change mode service request: following stopped");
///      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      enabled_ = false;
    }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  void publishBbox()
  {
    double x = (min_x_ + max_x_)/2;
    double y = (min_y_ + max_y_)/2;
    double z = (0 + max_z_)/2;

    double scale_x = (max_x_ - x)*2;
    double scale_y = (max_y_ - y)*2;
    double scale_z = (max_z_ - z)*2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = -y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    bboxpub_.publish( marker );
  }

  ros::Subscriber sub_;
///  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;

  ros::Publisher distpub_;
  ros::Publisher distpub1_;
  ros::Publisher distpub2_;
  ros::Publisher distpub3_;
  ros::Publisher distpub4_;
};

PLUGINLIB_DECLARE_CLASS(turtlebot_follower, TurtlebotFollower, turtlebot_follower::TurtlebotFollower, nodelet::Nodelet);

}
