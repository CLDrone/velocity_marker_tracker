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



int initialExpectPose;


void markerPoseReceived(const geometry_msgs::PoseStamped& msg)
{
   geometry_msgs::PoseStamped tag2cameraRDF;// = *msg;   //RDF - Right(x)-Down(y)-Front(z) Camera Coordinate
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
    double keep_distance;    
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
   
   
    tag2cameraRDF.pose.position = msg.pose.position;       
    Xc = tag2cameraRDF.pose.position.z;
    Yc = - tag2cameraRDF.pose.position.x;
    Zc = - tag2cameraRDF.pose.position.y;
    Dc = sqrt(Xc*Xc + Yc*Yc);
    Ay = asin(Yc/Dc);
    At = Ay + uavYawENU;
    
 
    //uavPosENU.pose.position.x = -(Dc*cos(At));
    //uavPosENU.pose.position.y = -(Dc*sin(At));
    //uavPosENU.pose.position.z = -(Zc);
   
    
   /* For Further Test*/
      uavPosENU.pose.position.x = -(Xc*cos(uavYawENU) - Yc*sin(uavYawENU));
      uavPosENU.pose.position.y = -(Xc*sin(uavYawENU) + Yc*cos(uavYawENU));
      uavPosENU.pose.position.z = -(Zc);
    /* */
    // Calculate Tag  Yaw
    	//uavPose.pose.orientation.x = tag2cameraRDF.pose.orientation.x;
	//uavPose.pose.orientation.y = tag2cameraRDF.pose.orientation.y;
	//uavPose.pose.orientation.z = tag2cameraRDF.pose.orientation.z;
	//uavPose.pose.orientation.w = tag2cameraRDF.pose.orientation.w;
	//tag2cameraRDF.pose.orientation = msg.pose.orientation;
	
        //tf::Quaternion quat_tag_;
	//tf::quaternionMsgToTF(tag2cameraRDF.pose.orientation, quat_tag_);
	//double roll_tag_, pitch_tag_, yaw_tag_;	
	//tf::Matrix3x3(quat_tag_).getRPY(roll_tag_, pitch_tag_, yaw_tag_);  
	//ROS_INFO("Tag RPY:[%0.3f, %0.3f, %0.3f]", (roll_tag_*180/3.1415926), pitch_tag_*180/3.1415926, yaw_tag_*180/3.1415926);
	//ROS_INFO("Tag XYZ:[%0.3f, %0.3f, %0.3f]", tag2cameraRDF.pose.position.x, tag2cameraRDF.pose.position.y, tag2cameraRDF.pose.position.z);
	//yaw_tag_ = roll_tag_ + 3.1415926/2;
 	
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
        double roll_, pitch_, yaw_;
        tf::Quaternion quat;
        
		if (!initialExpectPose)
		{
			
			
			nextPos.pose.position.x = uavPosENU.pose.position.x;
			nextPos.pose.position.y = uavPosENU.pose.position.y;			
			nextPos.pose.position.z = uavPosENU.pose.position.z;
			
						
			//tf::quaternionMsgToTF(uavPose.pose.orientation, quat);
			//tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
			roll_ = 0;
			pitch_ = 0;
			yaw_ = uavYawENU;
			quat = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
				
                        nextPos.pose.orientation.x = quat.x();
			nextPos.pose.orientation.y = quat.y();
			nextPos.pose.orientation.z = quat.z();
			nextPos.pose.orientation.w = quat.w();

			nextPos.header.stamp = ros::Time::now();
			
			//ROS_INFO("Current State: %d, Target Pos:[%0.3f, %0.3f, %0.3f] RPY:[%0.3f, %0.3f, %0.3f]", initialExpectPose, nextPos.pose.position.x, nextPos.pose.position.y, nextPos.pose.position.z, roll_*180/3.1415926, pitch_*180/3.1415926, yaw_*180/3.1415926);
			
			if ((nextPos.pose.position.x == 0) &&
				(nextPos.pose.position.y == 0) &&
				(nextPos.pose.position.z == 0))
			{
				initialExpectPose = 0;
			}
			else
			{					
				double distance_;
				distance_ = sqrt((uavPose.pose.position.x - nextPos.pose.position.x)+(uavPose.pose.position.y - nextPos.pose.position.y)+(uavPose.pose.position.z - nextPos.pose.position.z));
				
				//ROS_INFO("Current Distance %0.3f", distance_);
				
				if (distance_ > 0.8)
				{
					initialExpectPose = 0;
				}
				else
				{	
					nextPos.pose.position.x = -(Xc*cos(uavYawENU) - 0*sin(uavYawENU));
      					nextPos.pose.position.y = -(Xc*sin(uavYawENU) + 0*cos(uavYawENU));
      					nextPos.pose.position.z = -(0);
					
					//tf::quaternionMsgToTF(uavPose.pose.orientation, quat);
					//tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
					roll_ = 0;
					pitch_ = 0;
					
					//ROS_INFO("Current Yaw: %0.3f", yaw_*180/3.1415926);
					yaw_ = uavYawENU;
					quat = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
					
					//ROS_INFO("Target Yaw: %0.3f", yaw_*180/3.1415926);
					
                        		nextPos.pose.orientation.x = quat.x();
					nextPos.pose.orientation.y = quat.y();
					nextPos.pose.orientation.z = quat.z();
					nextPos.pose.orientation.w = quat.w();
					

					enuSetLocoalPositionPublisher.publish(nextPos); 
					initialExpectPose = 1;
				}
			}
		}
		else
		{	
			//nextPos.pose.position.x = -(Xc*cos(uavYawENU) - 0*sin(uavYawENU));
      			//nextPos.pose.position.y = -(Xc*sin(uavYawENU) + 0*cos(uavYawENU));
      			//nextPos.pose.position.z = -(0);
					
			//tf::quaternionMsgToTF(uavPose.pose.orientation, quat);
			//tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
			//roll_ = 0;
			//pitch_ = 0;
					
			//ROS_INFO("UAV Yaw: %0.3f", yaw_*180/3.1415926);
			//ROS_INFO("UAV Yaw ENU:%0.3f", uavYawENU*180/3.1415926);
			//ROS_INFO("Tag Yaw: %0.3f", yaw_tag_*180/3.1415926);

			//yaw_ = yaw_tag_+uavYawENU;
			//quat = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
					
			//ROS_INFO("Target Yaw: %0.3f", yaw_*180/3.1415926);
					
                        //nextPos.pose.orientation.x = quat.x();
			//nextPos.pose.orientation.y = quat.y();
			//nextPos.pose.orientation.z = quat.z();
			//nextPos.pose.orientation.w = quat.w();

			nextPos.header.stamp = ros::Time::now();
			enuSetLocoalPositionPublisher.publish(nextPos);
						
			//tf::quaternionMsgToTF(nextPos.pose.orientation, quat);
			//tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
			//ROS_INFO("Current State: %d, Target Pos:[%0.3f, %0.3f, %0.3f] RPY:[%0.3f, %0.3f, %0.3f]", initialExpectPose, nextPos.pose.position.x, nextPos.pose.position.y, nextPos.pose.position.z, roll_*180/3.1415926, pitch_*180/3.1415926, yaw_*180/3.1415926);
		
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
	
	//uavPose = *msg;	
	uavPose.pose.position.x = msg->pose.position.x;
	uavPose.pose.position.y = msg->pose.position.y;
	uavPose.pose.position.z = msg->pose.position.z;
	uavPose.pose.orientation.x = msg->pose.orientation.x;
	uavPose.pose.orientation.y = msg->pose.orientation.y;
	uavPose.pose.orientation.z = msg->pose.orientation.z;
	uavPose.pose.orientation.w = msg->pose.orientation.w;
	
	// Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
	tf::quaternionMsgToTF(uavPose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);
//	ROS_INFO("local_position/local: Pos:[%0.3f, %0.3f, %0.3f], RPY:[%0.3f, %0.3f, %0.3f]", uavPose.pose.position.x, uavPose.pose.position.y, uavPose.pose.position.z,uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);
	//ROS_INFO("Current UAV angles: roll=%0.3f, pitch=%0.3f, yaw=%0.3f", uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);	
	
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_marker_tracker_node");
  ros::NodeHandle nodeHandle;
  initialExpectPose = 0;

  bodyAxisVelocityPublisher = nodeHandle.advertise<geometry_msgs::TwistStamped>("/CLDrone/body_axis_velocity/cmd_vel",10);
  enuSetLocoalPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
  local_pos_pub_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

  ros::Subscriber markerPoseSubscriber = nodeHandle.subscribe("/aruco_single/pose",1000,markerPoseReceived);  
  ros::Subscriber markerPoseAprilSubscriber = nodeHandle.subscribe("/tag_detections_pose", 1000, markerPoseAprilReceived);
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
