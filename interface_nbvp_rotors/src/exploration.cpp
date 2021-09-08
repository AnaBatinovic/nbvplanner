/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>
#include <nbvplanner/volume_srv.h>

bool current_goal_reached = false;

void pointReachedCallback(std_msgs::Bool msg)
{
  if (msg.data)
  {
    ROS_INFO("Current goal point is reached!");
    current_goal_reached = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Publisher goals_pub = nh.advertise < geometry_msgs::PoseArray > ("nbvp/goals", 1);
  ros::Subscriber point_reached_sub = nh.subscribe("nbvp/point_reached", 1, &pointReachedCallback);
  ROS_INFO("Started exploration");

   double dt = 1.0;
  std::string ns = ros::this_node::getName();
  if (!ros::param::get(ns + "/nbvp/dt", dt)) {
    ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
              (ns + "/nbvp/dt").c_str());
    return -1;
  }
  // Get parent frame id
  std::string parent_frame_id = " ";
  nh.param<std::string>("exploration/parent_frame_id", parent_frame_id, "map"); 

  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  ros::Rate rate(1);
  nbvplanner::volume_srv volumeSrv;
  while (ros::ok()) {
    ros::spinOnce();
    if (current_goal_reached)
    {
      current_goal_reached = false;
      ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
      ROS_INFO("Iteration started at  %10.5fs",  (ros::Time::now()).toSec());
      nbvplanner::nbvp_srv planSrv;
      planSrv.request.header.stamp = ros::Time::now();
      planSrv.request.header.seq = iteration;
      planSrv.request.header.frame_id = parent_frame_id;
      if (ros::service::call("nbvplanner", planSrv)) {
        n_seq++;
        if (planSrv.response.path.size() == 0) {
          ros::Duration(1.0).sleep();
        }
        else {
          geometry_msgs::PoseArray goals;
          goals.header.seq = n_seq;
          goals.header.stamp = ros::Time::now();
          goals.header.frame_id = parent_frame_id;
          for (int i = 0; i < planSrv.response.path.size(); i++) {
            goals.poses.push_back(planSrv.response.path[i]);
          }
          goals_pub.publish(goals);
          // while (!current_goal_reached)
          // {
          //   ros::spinOnce();
          //   ros::Duration(dt).sleep();
          // }
          // current_goal_reached = false;
          // std::cout << "Current goal reached!" << std::endl;
        }
      } else {
        ROS_WARN_THROTTLE(1, "Planner not reachable");
        ros::Duration(1.0).sleep();
      }
    }
    //Call service for calculating and logging volume
    volumeSrv.request.header.stamp = ros::Time::now();
    volumeSrv.request.header.seq = iteration;
    volumeSrv.request.header.frame_id = parent_frame_id;
    if(ros::service::call("volume_service", volumeSrv)){
      // ROS_INFO("Volume updated.");
    }
    iteration++;
    rate.sleep();
  }
};

