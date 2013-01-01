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
#include <tf/transform_datatypes.h>

ros::Publisher bodyAxisVelocityPublisher;
geometry_msgs::TwistStamped vs;
ros::Publisher local_pos_pub_;
ros::Publisher enuSetLocoalPositionPublisher;
geometry_msgs::PoseStamped nextPos;
geometry_msgs::PoseStamped uavPose;
double uavRollENU, uavPitchENU, uavYawENU;

int initialExpectPose = 0;


void markerPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   geometry_msgs::PoseStamped tag2cameraRDF = *msg;   //RDF - Right(x)-Down(y)-Front(z) Camera Coordinate
   //geometry_msgs::Pose markerPose;
   // Front camera tracking
   //markerPose.position.x = dronePose.pose.position.x;
   //markerPose.position.z = dronePose.pose.position.y;
   //markerPose.position.y = dronePose.pose.position.z;
    
   // ROS_INFO_STREAM("marker pose" << "x:" << markerPose.position.x 
   //                        << "y:" << markerPose.position.y 
   //                        << "z:" << markerPose.position.z);
	
	
	
	
    double Xc, Yc, Zc, Dc, At;
    double Ay;    
    geometry_msgs::PoseStamped uavPosENU;
    
    /*  -- Original 
    Xt = tag2cameraRDF.pose.position.x;
    Yt = tag2cameraRDF.pose.position.z;
    Dc = sqrt(Xt*Xt + Yt*Yt);
    Ay = asin(Xt/Dc);
    At = Ay + uavYawENU;
               
    // Tag as origin
    uavPosENU.pose.position.x = -(Dc*cos(At));
    uavPosENU.pose.position.y = -(Dc*sin(At));
    uavPosENU.pose.position.z = -(-tag2cameraRDF.pose.position.y);
    */
    //uavPosENU.pose.position.x = -(tag2cameraRDF.pose.position.z);
    //uavPosENU.pose.position.y = -(-tag2cameraRDF.pose.position.x);
    //uavPosENU.pose.position.z = -(-tag2cameraRDF.pose.position.y);
   
   
       
    Xc = tag2cameraRDF.pose.position.z;
    Yc = - tag2cameraRDF.pose.position.x;
    Zc = - tag2cameraRDF.pose.position.y;
    Dc = sqrt(Xc*Xc + Yc*Yc);
    Ay = asin(Yc/Dc);
    At = Ay + uavYawENU;
    
    uavPosENU.pose.position.x = -(Dc*cos(At));
    uavPosENU.pose.position.y = -(Dc*sin(At));
    uavPosENU.pose.position.z = -(Zc);
   

    uavPosENU.header.stamp = ros::Time::now();   
    // Send the position to the FCU
    local_pos_pub_.publish(uavPosENU);
	
	// Publish Local Position Command to FCU
    /*Xt = 0;
    Yt = 0;
	Zt = -2;
    Dc = sqrt(Xt*Xt + Yt*Yt);	
    Ay = asin(Xt/Dc);
    At = Ay + uavYawENU;*/

	//nextPos.pose.position.x = -(Dc*cos(At));
	//nextPos.pose.position.y = -(Dc*sin(At));
	//nextPos.pose.position.z = -(-Yt);
    /*double expectYaw;
  	expectYaw = 0;
    nextPos.pose.position.x = -(cos(expectYaw)*Xt - sin(expectYaw)*Zt);
    nextPos.pose.position.y = -(sin(expectYaw)*Xt + cos(expectYaw)*Zt);
    nextPos.pose.position.z = -(-Yt);*/
        
		if (!initialExpectPose)
		{
			initialExpectPose = 1;
			nextPos.pose.position.x = uavPose.pose.position.x;
			nextPos.pose.position.y = uavPose.pose.position.y;			
			nextPos.pose.position.z = uavPose.pose.position.z;
			
			tf::Quaternion quat;
			double roll_, pitch_, yaw_;
			tf::quaternionMsgToTF(uavPose.pose.orientation, quat);
			tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
			roll_ = 0;
			pitch_ = 0;
			quat = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
				
                        nextPos.pose.orientation.x = quat.x();
			nextPos.pose.orientation.y = quat.y();
			nextPos.pose.orientation.z = quat.z();
			nextPos.pose.orientation.w = quat.w();

			nextPos.header.stamp = ros::Time::now();
			enuSetLocoalPositionPublisher.publish(nextPos); 
		}
		else
		{	
			nextPos.header.stamp = ros::Time::now();
			enuSetLocoalPositionPublisher.publish(nextPos);
		}

	  

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

void uavPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{

	uavPose = *msg;
	
	// Using ROS tf to get RPY angle from Quaternion
        tf::Quaternion quat;
	tf::quaternionMsgToTF(uavPose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);
		
	ROS_INFO("Current UAV angles: roll=%0.3f, pitch=%0.3f, yaw=%0.3f", uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);
			
	
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_marker_tracker_node");
  ros::NodeHandle nodeHandle;

  bodyAxisVelocityPublisher = nodeHandle.advertise<geometry_msgs::TwistStamped>("/CLDrone/body_axis_velocity/cmd_vel",10);
  enuSetLocoalPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  local_pos_pub_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

  ros::Subscriber markerPoseSubscriber = nodeHandle.subscribe("/aruco_single/pose",10,markerPoseReceived);  
  ros::Subscriber markerPoseAprilSubscriber = nodeHandle.subscribe("/tag_detections_pose", 10, markerPoseAprilReceived);
  ros::Subscriber uavPoseSubscriber = nodeHandle.subscribe("/mavros/local_position/pose", 1000, uavPoseReceived);
	
	
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
