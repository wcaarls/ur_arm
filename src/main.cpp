/*
 *    Universal Robots UR5 ROS node
 *    Copyright (C) 2012 Wouter Caarls
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

#include <ur_arm/ur_arm_node.h>

using namespace ur_arm;

/// Entry point for UR5 arm controller node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_arm_controller");

  std::string host = std::string("192.168.1.55");
  int port = 30002;

  ros::param::get("host", host);
  ros::param::get("port", port);

  Arm *arm = new Arm(host, port);
  ArmNode arm_node(arm);

  arm_node.spin();

  return 0;
}
