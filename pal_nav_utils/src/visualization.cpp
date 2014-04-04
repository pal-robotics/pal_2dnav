/**************************************************************************
 *
 *  Copyright (c) 2013, 2014 PAL Robotics SL
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

#include "pal_nav_utils/visualization.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

namespace pal
{

namespace exploration
{

  nav_msgs::OccupancyGrid createOccupancyGrid(const GridMap& map)
  {
    nav_msgs::OccupancyGrid msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";

    msg.info.width = map.width;
    msg.info.height = map.height;

    msg.info.resolution = map.resolution;
    msg.info.origin = map.origin;

    msg.data = map.data;

    return msg;
  }

  nav_msgs::OccupancyGrid createOccupancyGrid(const GridMap& map, const std::vector<int8_t>& cells)
  {
    GridMap tmp(map);
    tmp.data = cells;
    return createOccupancyGrid(tmp);
  }

  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<float>& cells)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;

    cloud.width = map.width;
    cloud.height = map.height;
    cloud.points.resize(map.size());
    for (index_t idx = 0; idx < map.size(); ++idx)
    {
      cloud.points[idx].x = map.xToCoord(idx % map.width);
      cloud.points[idx].y = map.yToCoord(idx / map.width);
      cloud.points[idx].z = cells[idx] / 30.0;
      cloud.points[idx].intensity = cells[idx];
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";

    return cloud_msg;
  }


  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<int8_t>& cells)
  {
    std::vector<float> tmp(cells.begin(), cells.end());
    return createPointCloud(map, tmp);
  }

  /*
   * Creates a PoseArray with arrows in each cell showing which paths
   * leave from there.
   *
   * prevmap: After "index_t p = prevmap[idx];", p contains the index of
   *          the previous node for the path loading to idx.
   */
  geometry_msgs::PoseArray createPoseArray(const GridMap& map, const std::vector<index_t>& prevmap)
  {
    geometry_msgs::PoseArray pose_array;

    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "map";

    pose_array.poses.reserve(map.size());
    for (index_t idx = 0; idx < map.size(); ++idx)
    {
      geometry_msgs::Pose pose;
      pose.position.x = map.xToCoord(idx % map.width);
      pose.position.y = map.yToCoord(idx / map.width);
      for (index_t nidx : map.getEightNeighbours(idx))
      {
        if (prevmap[nidx] == idx)
        {
          tf::Vector3 vec = map.getGradient(idx, nidx);
          double orientation = std::atan2(vec.y(), vec.x());
          pose.orientation = tf::createQuaternionMsgFromYaw(orientation);
          pose_array.poses.push_back(pose);
        }
      }
    }

    return pose_array;
  }

}  // namespace exploration
}  // namespace pal
