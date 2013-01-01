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
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

ros::Publisher bodyAxisVelocityPublisher;
geometry_msgs::TwistStamped vs;
ros::Publisher local_pos_pub_;
ros::Publisher enuSetLocoalPositionPublisher;
geometry_msgs::PoseStamped  nextPos;

void markerPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   geometry_msgs::PoseStamped camera2tagRDF = *msg;
   //geometry_msgs::Pose markerPose;
   // Front camera tracking
   //markerPose.position.x = dronePose.pose.position.x;
   //markerPose.position.z = dronePose.pose.position.y;
   //markerPose.position.y = dronePose.pose.position.z;
   
  // ROS_INFO_STREAM("marker pose" << "x:" << markerPose.position.x 
  //                        << "y:" << markerPose.position.y 
  //                        << "z:" << markerPose.position.z);
	
   geometry_msgs::PoseStamped uavPosENU;  // Tag as origin
   uavPosENU.pose.position.x = -(camera2tagRDF.pose.position.z);
   uavPosENU.pose.position.y = -(-camera2tagRDF.pose.position.x);
   uavPosENU.pose.position.z = -(-camera2tagRDF.pose.position.y);
   uavPosENU.header.stamp = ros::Time::now();
   // Send the position to the FCU
   local_pos_pub_.publish(uavPosENU);
	
	// Publish Local Position Command to FCU	
	nextPos.pose.position.x = -5;
	nextPos.pose.position.y = 0;
	nextPos.pose.position.z = 0;
	nextPos.header.stamp = ros::Time::now();
	enuSetLocoalPositionPublisher.publish(nextPos);
	
	/*
	vs.twist.linear.x = markerPose.position.y;
	vs.twist.linear.y = markerPose.position.z;
  
  

  if (abs(markerPose.position.x) < 0.1)
  {
    vs.twist.linear.x = 0;
  }

  if (abs(markerPose.position.y) < 0.1)
  {
    vs.twist.linear.y = 0;
  }

  if (vs.twist.linear.x > 1)
  {
      vs.twist.linear.x = 1;
  }
  if (vs.twist.linear.y > 1)
  {
      vs.twist.linear.y = 1;
  }
  if (vs.twist.linear.x < -1)
  {
      vs.twist.linear.x = -1;
  }
  if (vs.twist.linear.y < -1)
  {
      vs.twist.linear.y = -1;
  }

                */          

   

}

void markerPoseAprilReceived(const geometry_msgs::PoseArray::ConstPtr& msgArray)
{
	if (msgArray->poses.size() > 0)
	{
		// We have a target to process
    	geometry_msgs::Pose msg = msgArray->poses.at(0);
		
        // copy the message header since we need it when we forward it on to mavros
		geometry_msgs::PoseStamped vision_pose_;
        vision_pose_.header = msgArray->header;
		
		ROS_INFO_STREAM("Received Apriltag Pose! [" << "x: " << msg.position.x 
                          << " y: " << msg.position.y 
                          << " z: " << msg.position.z << "]");
		
		vision_pose_.pose.position.x = msg.position.x;
		vision_pose_.pose.position.y = msg.position.y;
		vision_pose_.pose.position.z = msg.position.z;
		vision_pose_.header.stamp = ros::Time::now();
		// Send the position to the FCU
		local_pos_pub_.publish(vision_pose_);
		
		
		
	}
	else
	{
		
	}
	
}







int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_marker_tracker_node");
  ros::NodeHandle nodeHandle;

 // bodyAxisVelocityPublisher = nodeHandle.advertise<geometry_msgs::TwistStamped>("/CLDrone/body_axis_velocity/cmd_vel",10);
  enuSetLocoalPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  local_pos_pub_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

  ros::Subscriber markerPoseSubscriber = nodeHandle.subscribe("/aruco_single/pose",10,markerPoseReceived);  
  ros::Subscriber markerPoseAprilSubscriber = nodeHandle.subscribe("/tag_detections_pose", 10, markerPoseAprilReceived);

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
