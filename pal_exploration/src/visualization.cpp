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

#include "pal_exploration/visualization.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

#include "pal_exploration/frontiers.h"

using namespace pal::nav;

namespace pal
{

namespace exploration
{

  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<Frontier>& frontiers)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    cloud.points.clear();
    uint32_t i = 0;
    for (const Frontier& frontier : frontiers)
    {
      pcl::PointXYZRGB point;
      point.r = 51 * (1 + i % 4);
      point.g = 51 * (1 + (i / 4) % 4);
      point.b = 51 * (1 + i / 16);
      i = (i + 1) % 64;
      for (const FrontierCell& cell : frontier.cells())
      {
        point.x = cell.position.x();
        point.y = cell.position.y();
        point.z = 0;
        cloud.points.push_back(point);
      }
    }

    // PCL wants "width * height == points.size()".
    // They don't seem to be used otherwise.
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";

    return cloud_msg;
  }

}  // namespace exploration
}  // namespace pal
