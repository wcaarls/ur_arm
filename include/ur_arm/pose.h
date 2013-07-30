/*
 *    Universal Robots UR5 tool and joint pose classes
 *    Copyright (C) 2012 Jethro Tan, Maarten de Vries, Wouter Caarls
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

#ifndef UNIVERSAL_ROBOTS_POSE_H_
#define UNIVERSAL_ROBOTS_POSE_H_

#include <stdexcept>

namespace ur_arm
{
  struct ToolPosition
  {
      double x;
      double y;
      double z;
      double roll;
      double pitch;
      double yaw;

      ToolPosition() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) { }

      ToolPosition(double x, double y, double z, double roll, double pitch, double yaw) :
        x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) { }

      ToolPosition & operator+=(const ToolPosition & b)
      {
        x += b.x;
        y += b.y;
        z += b.z;
        roll += b.roll;
        pitch += b.pitch;
        yaw += b.yaw;
        return *this;
      }

      ToolPosition operator+(const ToolPosition & b) const
      {
        ToolPosition result = *this;
        return result += b;
      }

      ToolPosition & operator-=(const ToolPosition & b)
      {
        x -= b.x;
        y -= b.y;
        z -= b.z;
        roll -= b.roll;
        pitch -= b.pitch;
        yaw -= b.yaw;
        return *this;
      }

      ToolPosition operator-(const ToolPosition & b) const
      {
        ToolPosition result = *this;
        return result -= b;
      }

      ToolPosition & operator*=(const double b)
      {
        x *= b;
        y *= b;
        z *= b;
        roll *= b;
        pitch *= b;
        yaw *= b;
        return *this;
      }

      ToolPosition operator*(const double b)
      {
        ToolPosition result = *this;
        return result *= b;
      }

      double &operator[](const size_t index)
      {
        switch (index)
        {
          case 0: return x;
          case 1: return y;
          case 2: return z;
          case 3: return roll;
          case 4: return pitch;
          case 5: return yaw;
          default:
            throw std::out_of_range("The requested index is out of range.");
        }
      }
  };

  /// A list of tool speeds in [m/s] and [rad/s]
  typedef ToolPosition ToolTwist;

  /// A list of tool accelerations in [m/s^2] and [rad/s^2]
  typedef ToolPosition ToolAccelerations;

  /// A list of joint positions in [rad].
  struct JointAngles
  {
      double base;
      double shoulder;
      double elbow;
      double wrist1;
      double wrist2;
      double wrist3;

      JointAngles() : base(0), shoulder(0), elbow(0), wrist1(0), wrist2(0), wrist3(0) { }

      JointAngles(double base, double shoulder, double elbow, double wrist1,
          double wrist2, double wrist3) :
        base(base), shoulder(shoulder), elbow(elbow), wrist1(wrist1), wrist2(
            wrist2), wrist3(wrist3)
      {
      }

      JointAngles & operator+=(const JointAngles & b)
      {
        base += b.base;
        shoulder += b.shoulder;
        elbow += b.elbow;
        wrist1 += b.wrist1;
        wrist2 += b.wrist2;
        wrist3 += b.wrist3;
        return *this;
      }

      JointAngles operator+(const JointAngles & b) const
      {
        JointAngles result = *this;
        return result += b;
      }

      JointAngles & operator-=(const JointAngles & b)
      {
        base -= b.base;
        shoulder -= b.shoulder;
        elbow -= b.elbow;
        wrist1 -= b.wrist1;
        wrist2 -= b.wrist2;
        wrist3 -= b.wrist3;
        return *this;
      }

      JointAngles operator-(const JointAngles & b) const
      {
        JointAngles result = *this;
        return result -= b;
      }

      JointAngles & operator*=(const double b)
      {
        base *= b;
        shoulder *= b;
        elbow *= b;
        wrist1 *= b;
        wrist2 *= b;
        wrist3 *= b;
        return *this;
      }

      JointAngles operator*(const double b)
      {
        JointAngles result = *this;
        return result *= b;
      }

      double &operator[](const size_t index)
      {
        switch (index)
        {
          case 0: return base;
          case 1: return shoulder;
          case 2: return elbow;
          case 3: return wrist1;
          case 4: return wrist2;
          case 5: return wrist3;
          default:
            throw std::out_of_range("The requested index is out of range.");
        }
      }
  };

  /// A list of joint speeds in [rad/s]
  typedef JointAngles JointSpeeds;

  /// A list of joint accelerations in [rad/s^2]
  typedef JointAngles JointAccelerations;
}

#endif /* UNIVERSAL_ROBOTS_POSE_H_ */
