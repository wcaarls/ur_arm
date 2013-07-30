/*
 *    Universal Robots UR5 controller
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

#ifndef UNIVERSAL_ROBOTS_ARM_H_
#define UNIVERSAL_ROBOTS_ARM_H_

#include <stdio.h>
#include <endian.h>

#include <ur_arm/pose.h>

namespace ur_arm
{
  inline double bedtoh(const double &x)
  {
    double temp;
    *((uint64_t*)(&temp)) = be64toh(*((uint64_t*)&x));
    return temp;
  }

  inline float beftoh(const float &x)
  {
    float temp;
    *((uint32_t*)(&temp)) = be32toh(*((uint32_t*)&x));
    return temp;
  }

  class Joint
  {
    public:
      double pos1, pos2;
      double vel;
      float current, voltage, temperature, unknown;
      unsigned char status;

    public:
      void fixByteOrder()
      {
        pos1        = bedtoh(pos1);
        pos2        = bedtoh(pos2);
        vel         = bedtoh(vel);
        current     = beftoh(current);
        voltage     = beftoh(voltage);
        temperature = beftoh(temperature);
        unknown     = beftoh(unknown);
      }

  } __attribute__((packed));

  class Packet
  {
    public:
      double time;
      unsigned char preamble[27];
      Joint joint[6];

    public:
      void fixByteOrder()
      {
        time = bedtoh(time);
        for (int ii=0; ii < 6; ++ii)
          joint[ii].fixByteOrder();
      }
  } __attribute__((packed));

  class Arm
  {
    protected:
      std::string host_;
      int port_;

      int socket_;
      JointAngles position_;
      JointSpeeds speed_;

    public:
    Arm(std::string host, int port) : host_(host), port_(port), socket_(0)
    {
      init();
    }

    ~Arm()
    {
      if (socket_)
        close(socket_);
    }

    /// Initialize
    /** \note Called during construction */
    void init();

    void moveJoints(JointAngles position, double speed, double accel);
    void moveTool(ToolPosition position, double speed, double accel);
    void setJointSpeeds(JointSpeeds speed, double accel, double time);
    void setToolSpeed(ToolTwist speed, double accel, double time);

    /// Read feedback from robot
    void update();

    JointAngles getJointAngles()
    {
      return position_;
    }

    JointSpeeds getJointSpeeds()
    {
      return speed_;
    }
  };
}

#endif /* UNIVERSAL_ROBOTS_ARM_H_ */
