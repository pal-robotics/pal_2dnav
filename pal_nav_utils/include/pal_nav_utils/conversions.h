/**************************************************************************
 *
 *  Copyright (c) 2014 PAL Robotics SL
 *
 *  Permission to use, copy, modify, and/or distribute this software for
 *  any purpose with or without fee is hereby granted, provided that the
 *  above copyright notice and this permission notice appear in all
 *  copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
 *  REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
 *  SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 *  OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  Authors:
 *   - Siegfried-A. Gevatter Pujals <siegfried.gevatter@pal-robotics.com>
 *
 *************************************************************************/

#ifndef PAL_NAV_UTILS_CONVERSIONS_
#define PAL_NAV_UTILS_CONVERSIONS_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

namespace pal
{

namespace nav
{

inline geometry_msgs::Point makePoint(double x, double y, double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

inline geometry_msgs::Point makePoint(const geometry_msgs::Vector3& v)
{
  return makePoint(v.x, v.y, v.z);
}

inline geometry_msgs::Vector3 makeVector3(double x, double y, double z)
{
  geometry_msgs::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

inline geometry_msgs::Vector3 makeVector3(const geometry_msgs::Point& p)
{
  return makeVector3(p.x, p.y, p.z);
}

inline geometry_msgs::Pose makePose(double x, double y, double z, double yaw)
{
  geometry_msgs::Pose p;
  p.position = makePoint(x, y, z);
  p.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return p;
}

inline geometry_msgs::Pose makePose(const geometry_msgs::Point& point, double yaw)
{
  geometry_msgs::Pose p;
  p.position = point;
  p.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return p;
}

inline geometry_msgs::Pose makePose(const geometry_msgs::Point& point, const geometry_msgs::Quaternion& q)
{
  geometry_msgs::Pose p;
  p.position = point;
  p.orientation = q;
  return p;
}

inline geometry_msgs::Pose makePose(const geometry_msgs::Transform& transf)
{
  geometry_msgs::Pose p;
  p.position = makePoint(transf.translation);
  p.orientation = transf.rotation;
  return p;
}

inline geometry_msgs::Transform makeTransform(const geometry_msgs::Pose p)
{
  geometry_msgs::Transform transf;
  transf.translation = makeVector3(p.position);
  transf.rotation = p.orientation;
  return transf;
}

}  // namespace nav
}  // namespace pal

#endif  // PAL_NAV_UTILS_CONVERSIONS_
