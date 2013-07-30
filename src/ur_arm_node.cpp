/*
 *    Universal Robots UR5 ROS node
 *    Copyright (C) 2012 Wouter Caarls, Jethro Tan, Maarten de Vries
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ur_arm/Joints.h>
#include <ur_arm/ur_arm_node.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace ur_arm;

void ArmNode::jointPositionCallback(const ur_arm::Joints::ConstPtr &msg)
{
  JointAngles params;
  params.base = msg->base;
  params.shoulder = msg->shoulder;
  params.elbow = msg->elbow;
  params.wrist1 = msg->wrist1;
  params.wrist2 = msg->wrist2;
  params.wrist3 = msg->wrist3;

  lock();
  if (command_) delete command_;
  command_ = new MoveJointsCommand(arm_, params, speed_, accel_);
  unlock();
}

void ArmNode::toolPositionCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  ToolPosition params;
  params.x = msg->position.x;
  params.y = msg->position.y;
  params.z = msg->position.z;

  quaternionToRpy(msg->orientation, params.roll, params.pitch, params.yaw);
  lock();
  if (command_) delete command_;
  command_ = new MoveToolCommand(arm_, params, speed_, accel_);
  unlock();
}

void ArmNode::jointVelocityCallback(const ur_arm::Joints::ConstPtr &msg)
{
  JointSpeeds params;
  params.base = msg->base;
  params.shoulder = msg->shoulder;
  params.elbow = msg->elbow;
  params.wrist1 = msg->wrist1;
  params.wrist2 = msg->wrist2;
  params.wrist3 = msg->wrist3;

  lock();
  if (command_) delete command_;
  command_ = new JointSpeedsCommand(arm_, params, accel_, (1.0/frequency_)*2);
  unlock();
}

void ArmNode::toolVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  ToolTwist params;
  params.x = msg->linear.x;
  params.y = msg->linear.y;
  params.z = msg->linear.z;
  params.roll = msg->angular.x;
  params.pitch = msg->angular.y;
  params.yaw = msg->angular.z;


  lock();
  if (command_) delete command_;
  command_ = new ToolSpeedCommand(arm_, params, accel_, (1.0/frequency_)*2);
  unlock();

}

void ArmNode::toolTaskFrameVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	JointAngles angles = arm_->getJointAngles();

	double q[6];
	q[0] = angles.base;
	q[1] = angles.shoulder;
	q[2] = angles.elbow;
	q[3] = angles.wrist1;
	q[4] = angles.wrist2;
	q[5] = angles.wrist3;

	forkin_solver.solveForwardKin(q);

	Eigen::MatrixXd vel_vec_linear(3,1);
	Eigen::MatrixXd vel_vec_angular(3,1);
	Eigen::MatrixXd rot_mat(3,3);
	Eigen::MatrixXd res_lin(3,1);
	Eigen::MatrixXd res_ang(3,1);

	vel_vec_linear(0,0) = 	msg->linear.x;
	vel_vec_linear(1,0) = 	msg->linear.y;
	vel_vec_linear(2,0) = 	msg->linear.z;

	vel_vec_angular(0,0) = msg->angular.x;
	vel_vec_angular(1,0) = msg->angular.y;
	vel_vec_angular(2,0) = msg->angular.z;

	for(int i = 0; i<3; i++)
		for(int j=0; j<3; j++)
			rot_mat(i,j) = forkin_solver.R[i][j];

	res_lin = rot_mat*vel_vec_linear;
	res_ang = rot_mat*vel_vec_angular;

  ToolTwist params;
  params.x = res_lin(0,0);
  params.y = res_lin(1,0);
  params.z = res_lin(2,0);
  params.roll = res_ang(0,0);
  params.pitch = res_ang(1,0);
  params.yaw = res_ang(2,0);

  lock();

  if (command_) delete command_;
  command_ = new ToolSpeedCommand(arm_, params, accel_, (1.0/frequency_)*2.0);
  unlock();

}

void ArmNode::init()
{
  // Read parameters
  nh_.getParam("frequency", frequency_);
  nh_.getParam("speed", speed_);
  nh_.getParam("acceleration", accel_);

  //home();
  srvsrvr_homing_= nh_.advertiseService("cmd_home", &ArmNode::homingCallback, this);
  joint_vel_sub_ = nh_.subscribe("cmd_joint_vel", 1, &ArmNode::jointVelocityCallback, this);
  tool_vel_sub_ = nh_.subscribe("cmd_tool_vel", 1, &ArmNode::toolVelocityCallback, this);
  toolTF_vel_sub_ = nh_.subscribe("cmd_tool_tf_vel", 1, &ArmNode::toolTaskFrameVelocityCallback, this);
  joint_pos_sub_ = nh_.subscribe("cmd_joint_pos", 1, &ArmNode::jointPositionCallback, this);
  tool_pos_sub_ = nh_.subscribe("cmd_tool_pos", 1, &ArmNode::toolPositionCallback, this);
  joint_pos_pub_ = nh_.advertise<ur_arm::Joints>("joint_pos", 1);
  joint_vel_pub_ = nh_.advertise<ur_arm::Joints>("joint_vel", 1);
  tool_pos_pub_ = nh_.advertise<geometry_msgs::Pose>("tool_pos", 1);


}

bool ArmNode::homingCallback(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res){
	return home();
}

bool ArmNode::home(){
  JointAngles params;
  std_msgs::Bool msg_arm_homing_status;
  if(nh_.getParam("homing_joint_pos1",params.base)
		  &&nh_.getParam("homing_joint_pos2",params.shoulder)
		  &&nh_.getParam("homing_joint_pos3",params.elbow)
		  &&nh_.getParam("homing_joint_pos4",params.wrist1)
		  &&nh_.getParam("homing_joint_pos5",params.wrist2)
		  &&nh_.getParam("homing_joint_pos6",params.wrist3)){
	  params.base = params.base/180.0*3.141592;
	  params.shoulder = params.shoulder/180.0*3.141592;
	  params.elbow = params.elbow/180.0*3.141592;
	  params.wrist1 = params.wrist1/180.0*3.141592;
	  params.wrist2 = params.wrist2/180.0*3.141592;
	  params.wrist3 = params.wrist3/180.0*3.141592;


	  lock();
	  if (command_) delete command_;
	  command_ = new MoveJointsCommand(arm_, params, speed_, accel_);
	  unlock();
	  command_->apply();
	  //XXX: It only wait for 3 seconds and then assumes that the homing is done. The position errors of the joints can be checked instead.
	  ros::Rate homing_check_rate(0.5);
	  while(ros::ok()){
		  arm_->update();
		  JointAngles angles = arm_->getJointAngles();
		  if(fabs(params.base-angles.base) <0.01 &&
			 fabs(params.shoulder-angles.shoulder) <0.01 &&
			 fabs(params.elbow-angles.elbow) <0.01 &&
			 fabs(params.wrist1-angles.wrist1) <0.01 &&
			 fabs(params.wrist2-angles.wrist2) <0.01 &&
			 fabs(params.wrist3-angles.wrist3) <0.01){
			  ros::spinOnce();
			  break;
		  }
		  ros::spinOnce();
		  homing_check_rate.sleep();
	  }
	return true;
  }
  else{
	  ROS_ERROR("[ur_arm]: Homing is unsuccessful. The parameters are not set.");
	  return false;
  }
}

void ArmNode::publishJointInfo()
{
  JointAngles angles = arm_->getJointAngles();
  ur_arm::Joints posmsg;

  posmsg.base = angles.base;
  posmsg.shoulder = angles.shoulder;
  posmsg.elbow = angles.elbow;
  posmsg.wrist1 = angles.wrist1;
  posmsg.wrist2 = angles.wrist2;
  posmsg.wrist3 = angles.wrist3;

  joint_pos_pub_.publish(posmsg);

  JointSpeeds speeds = arm_->getJointSpeeds();
  ur_arm::Joints velmsg;

  velmsg.base = speeds.base;
  velmsg.shoulder = speeds.shoulder;
  velmsg.elbow = speeds.elbow;
  velmsg.wrist1 = speeds.wrist1;
  velmsg.wrist2 = speeds.wrist2;
  velmsg.wrist3 = speeds.wrist3;

  joint_vel_pub_.publish(velmsg);
}

void ArmNode::publishToolPos()
{

	JointAngles angles = arm_->getJointAngles();

	double q[6];
	q[0] = angles.base;
	q[1] = angles.shoulder;
	q[2] = angles.elbow;
	q[3] = angles.wrist1;
	q[4] = angles.wrist2;
	q[5] = angles.wrist3;
	forkin_solver.solveForwardKin(q);

	geometry_msgs::Pose msg_pose;
	msg_pose.position.x = forkin_solver.T[0];
	msg_pose.position.y = forkin_solver.T[1];
	msg_pose.position.z = forkin_solver.T[2];

	double yaw, pitch, roll;
	yaw = std::atan2(forkin_solver.R[1][0],forkin_solver.R[0][0]);
	pitch = std::atan2(-forkin_solver.R[2][0],std::sqrt(std::pow(forkin_solver.R[2][1],2.0)+std::pow(forkin_solver.R[2][2],2.0)));
	roll = std::atan2(forkin_solver.R[2][1],forkin_solver.R[2][2]);
	msg_pose.orientation.x = roll;
	msg_pose.orientation.y = pitch;
	msg_pose.orientation.z = yaw;
	msg_pose.orientation.w = 0;
	tool_pos_pub_.publish(msg_pose);


}

void ArmNode::spin()
{
  ros::Rate loop_rate(frequency_);

  while (ros::ok())
  {

    arm_->update();
    lock();
    if (command_)
      command_->apply();
    unlock();
    publishJointInfo();
    publishToolPos();
    ros::spinOnce();
    loop_rate.sleep();

  }
}
