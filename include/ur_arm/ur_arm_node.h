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

#ifndef UNIVERSAL_ROBOTS_ARM_NODE_H_
#define UNIVERSAL_ROBOTS_ARM_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <ur_arm/Joints.h>


#include <ur_arm/pose.h>

#include <ur_arm/ur_arm.h>

#include <std_msgs/Bool.h>
#include "ur_arm/ur_arm_forkin.h"

namespace ur_arm
{
  inline void quaternionToRpy(const geometry_msgs::Quaternion &q,
      double &roll, double &pitch, double &yaw)
  {
    roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    pitch = std::asin(2 * (q.w * q.y - q.x * q.z));
    yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }
  
  inline void rpyToQuaternion(double roll, double pitch, double yaw,
      geometry_msgs::Quaternion &q)
  {
    double cr2 = std::cos(roll/2), sr2 = std::sin(roll/2);
    double cp2 = std::cos(pitch/2), sp2 = std::sin(pitch/2);
    double cy2 = std::cos(yaw/2), sy2 = std::sin(yaw/2);
    
    q.w = cr2*cp2*cy2 + sr2*sp2*sy2;
    q.x = sr2*cp2*cy2 - cr2*sp2*sy2;
    q.y = cr2*sp2*cy2 + sr2*cp2*sy2;
    q.z = cr2*cp2*sy2 - sr2*sp2*cy2;
  }    

  class Command
  {
    public:
      virtual void apply() = 0;
  };

  class MoveJointsCommand : public Command
  {
    private:
      Arm *arm_;
      JointAngles params_;
      double speed_, accel_;
      bool done_;

    public:
      MoveJointsCommand(Arm *arm, JointAngles params, double speed, double accel) :
        arm_(arm), params_(params), speed_(speed), accel_(accel), done_(false) { }

      void apply()
      {
        if (!done_)
        {
          arm_->moveJoints(params_, speed_, accel_);
          done_ = true;
        }
      }
  };

  class MoveToolCommand : public Command
  {
    private:
      Arm *arm_;
      ToolPosition params_;
      double speed_, accel_;
      bool done_;

    public:
      MoveToolCommand(Arm *arm, ToolPosition params, double speed, double accel) :
        arm_(arm), params_(params), speed_(speed), accel_(accel), done_(false) { }

      void apply()
      {
        if (!done_)
        {
          arm_->moveTool(params_, speed_, accel_);
          done_ = true;
        }
      }
  };

  class JointSpeedsCommand : public Command
  {
    private:
      Arm *arm_;
      JointSpeeds params_;
      double accel_, time_;

    public:
      JointSpeedsCommand(Arm *arm, JointSpeeds params, double accel, double time) :
        arm_(arm), params_(params), accel_(accel), time_(time) { }

      void apply()
      {
        arm_->setJointSpeeds(params_, accel_, time_);
      }
  };

  class ToolSpeedCommand : public Command
  {
    private:
      Arm *arm_;
      ToolTwist params_;
      double accel_, time_;

    public:
      ToolSpeedCommand(Arm *arm, ToolTwist params, double accel, double time) :
        arm_(arm), params_(params), accel_(accel), time_(time) { }

      void apply()
      {
        arm_->setToolSpeed(params_, accel_, time_);
      }
  };

  class ArmNode
  {
    protected:
      ros::NodeHandle nh_;
      ros::Subscriber joint_vel_sub_, tool_vel_sub_, joint_pos_sub_, tool_pos_sub_, toolTF_vel_sub_,sub_homing_command_;
      ros::Publisher joint_pos_pub_, joint_vel_pub_, tool_pos_pub_;
      ros::ServiceServer srvsrvr_homing_;
      double frequency_, speed_, accel_;

      Arm *arm_;
      Command *command_;
      pthread_mutex_t mutex_; ///< Mutual exclusion lock
      URForwardKinSolver forkin_solver;

    protected:
      /// Called when a new joint position command is received
      /**
       * Sends the new joint position to the arm controller
       * \param msg Pointer to a ur_arm/Joints message, containing the position in [rad]
       * \note Interrupting position commands can cause the robot to jerk.
       */
      void jointPositionCallback(const ur_arm::Joints::ConstPtr &msg);
      void publishToolPos();
      void toolTaskFrameVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
      /// Called when a new tool position command is received
      /**
       * Sends the new tool position to the arm controller
       * \param msg Pointer to a geometry_msgs/Pose message, containing the position in [m] and [rad]
       * \note Interrupting position commands can cause the robot to jerk.
       */
      void toolPositionCallback(const geometry_msgs::Pose::ConstPtr &msg);

      /// Called when a new joint velocity command is received
      /**
       * Sends the new velocity to the arm controller
       * \param msg Pointer to a ur_arm/Joints message, containing the velocity in [rad/s]
       */
      void jointVelocityCallback(const ur_arm::Joints::ConstPtr &msg);

      /// Called when a new tool velocity command is received
      /**
       * Sends the new velocity to the arm controller
       * \param msg Pointer to a geometry_msgs/Twist message, containing the velocity in [m/s] and [rad/s]
       */
      void toolVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);

      void publishJointInfo();

      bool home();
      bool homingCallback(std_srvs::Empty::Request &req,
      		std_srvs::Empty::Response &res);

      /// Lock
      void lock() { pthread_mutex_lock(&mutex_); }

      /// Unlock
      void unlock() { pthread_mutex_unlock(&mutex_); }

    public:
      ArmNode(Arm *arm) :
        nh_("~"), frequency_(20), speed_(0.1), accel_(0.1), arm_(arm), command_(NULL)
      {
        pthread_mutex_init(&mutex_, NULL);

        init();
      }

      ~ArmNode()
      {
        pthread_mutex_destroy(&mutex_);

        delete arm_;
        nh_.shutdown();
      }

      /// Initialize the arm controller
      /** \note Called during construction */
      void init();

      /// Await and process commands
      void spin();
  };
}

#endif /* UNIVERSAL_ROBOTS_ARM_NODE_H_ */
