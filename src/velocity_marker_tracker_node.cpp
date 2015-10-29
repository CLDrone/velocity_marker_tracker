/****************************************************************************
 *
 *   Copyright (c) 2015 Crossline Drone Project Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name CLDrone nor Crossline Drone nor the names of its c
 *    ontributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>

ros::Publisher bodyAxisVelocityPublisher;
geometry_msgs::TwistStamped vs;


void markerPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   geometry_msgs::PoseStamped dronePose = *msg;
   geometry_msgs::Pose markerPose;
   // Front camera tracking
   markerPose.position.x = dronePose.pose.position.z;
   markerPose.position.z = -dronePose.pose.position.y;
   markerPose.position.y = dronePose.pose.position.x;
   
   ROS_INFO_STREAM("marker pose" << "x:" << markerPose.position.x 
                          << "y:" << markerPose.position.y 
                          << "z:" << markerPose.position.z);

  vs.twist.linear.y = markerPose.position.y;
  vs.twist.linear.z = markerPose.position.z;

  if (abs(markerPose.position.y) < 0.1)
  {
    vs.twist.linear.y = 0;
  }

  if (abs(markerPose.position.z) < 0.1)
  {
    vs.twist.linear.z = 0;
  }

  if (vs.twist.linear.y > 1)
  {
      vs.twist.linear.y = 1;
  }
  if (vs.twist.linear.z > 1)
  {
      vs.twist.linear.z = 1;
  }
  if (vs.twist.linear.y < -1)
  {
      vs.twist.linear.y = -1;
  }
  if (vs.twist.linear.z < -1)
  {
      vs.twist.linear.z = -1;
  }

                          

   

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_marker_tracker_node");
  ros::NodeHandle nodeHandle;

  bodyAxisVelocityPublisher = nodeHandle.advertise<geometry_msgs::TwistStamped>("/CLDrone/body_axis_velocity/cmd_vel",10);
  ros::Subscriber markerPoseSubscriber = nodeHandle.subscribe("/aruco_single/pose",10,markerPoseReceived);


  ros::Rate loopRate(10.0);
  while(ros::ok())
  {
    vs.header.seq++;
	  vs.header.stamp = ros::Time::now();

	  
	  bodyAxisVelocityPublisher.publish(vs);
	  
    
  	ros::spinOnce();
  	loopRate.sleep();
  }

  return 0;

}