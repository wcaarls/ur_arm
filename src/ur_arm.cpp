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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <poll.h>
#include <endian.h>

#include "ur_arm/ur_arm.h"

#define BUFSIZE 1024

using namespace ur_arm;

void Arm::init()
{
  int optval = 1;

  struct sockaddr_in arm_addr;
  struct hostent *server;

  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

  bzero((char *) &arm_addr, sizeof(arm_addr));
  arm_addr.sin_family = AF_INET;
  server = gethostbyname(host_.c_str());
  if (!server)
    throw("unknown universal robots arm host name");

  bcopy((char *)server->h_addr, (char *)&arm_addr.sin_addr.s_addr, server->h_length);
  arm_addr.sin_port = htons(port_);

  if (connect(socket_,(struct sockaddr *) &arm_addr,sizeof(arm_addr)) < 0)
    throw("error connecting to universal robots arm");
}

void Arm::moveJoints(JointAngles position, double speed, double accel)
{
  char buf[255];

  snprintf(buf, 255, "movej([%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f], %5.2f, %5.2f)\n",
      position.base, position.shoulder, position.elbow,
      position.wrist1, position.wrist2, position.wrist3,
      speed, accel);

#ifdef DEBUG
  puts(buf);
#endif

  if (write(socket_, buf, strlen(buf)) != (ssize_t)strlen(buf))
    throw("couldn't send command to universal robots arm");

}

void Arm::moveTool(ToolPosition position, double speed, double accel)
{
  char buf[255];

  snprintf(buf, 255, "movel(p[%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f], %5.2f, %5.2f)\n",
      position.x, position.y, position.z,
      position.roll, position.pitch, position.yaw,
      speed, accel);

#ifdef DEBUG
  puts(buf);
#endif

  if (write(socket_, buf, strlen(buf)) != (ssize_t)strlen(buf))
    throw("couldn't send command to universal robots arm");
}

void Arm::setJointSpeeds(JointSpeeds speed, double accel, double time)
{
  char buf[255];

  snprintf(buf, 255, "speedj([%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f], %5.2f, %5.2f)\n",
      speed.base, speed.shoulder, speed.elbow,
      speed.wrist1, speed.wrist2, speed.wrist3,
      accel, time);

#ifdef DEBUG
  puts(buf);
#endif

  if (write(socket_, buf, strlen(buf)) != (ssize_t)strlen(buf))
    throw("couldn't send command to universal robots arm");
}

void Arm::setToolSpeed(ToolTwist speed, double accel, double time)
{
  char buf[255];

  snprintf(buf, 255, "speedl([%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f], %5.2f, %5.2f)\n",
      speed.x, speed.y, speed.z,
      speed.roll, speed.pitch, speed.yaw,
      accel, time);

#ifdef DEBUG
  puts(buf);
#endif

  if (write(socket_, buf, strlen(buf)) != (ssize_t)strlen(buf))
    throw("couldn't send command to universal robots arm");
}

void Arm::update()
{
  unsigned char buf[BUFSIZE];

  struct pollfd pfd;
  pfd.fd = socket_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  while (poll(&pfd, 1, 1))
  {
    uint32_t msgsize;

    if (read(socket_, &msgsize, 4) != 4)
      throw("couldn't read universal robots arm packet size");

    msgsize = be32toh(msgsize)-4;

    if (msgsize < BUFSIZE && msgsize >= sizeof(Packet))
    {
      if (read(socket_, buf, msgsize) != (ssize_t)msgsize)
        throw("couldn't read universal robots arm packet");

      Packet *packet = (Packet*) buf;
      packet->fixByteOrder();

      for (int ii=0; ii < 6; ++ii)
      {
        position_[ii] = packet->joint[ii].pos1;
        speed_[ii] = packet->joint[ii].vel;
      }
    }
    else
      throw std::out_of_range("universal robots arm packet size out of bounds");
  }
}
