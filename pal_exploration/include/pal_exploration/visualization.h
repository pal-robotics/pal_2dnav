/**************************************************************************
**
**  visualization.h
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 07-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef PAL_EXPLORATION_VISUALIZATION_
#define PAL_EXPLORATION_VISUALIZATION_

#include <vector>
#include <string>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pal_exploration/grid_map.h>

namespace pal
{

namespace exploration
{

  nav_msgs::OccupancyGrid createOccupancyGrid(const GridMap& map);

  nav_msgs::OccupancyGrid createOccupancyGrid(const GridMap& map, const std::vector<int8_t>& cells);

  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<float>& cells);
  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<int8_t>& cells);

  class Frontier;
  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<Frontier>& frontiers);

  /*
   * Creates a PoseArray with arrows in each cell showing which paths
   * leave from there.
   *
   * prevmap: After "index_t p = prevmap[idx];", p contains the index of
   *          the previous node for the path loading to idx.
   */
  geometry_msgs::PoseArray createPoseArray(const GridMap& map, const std::vector<index_t>& prevmap);

  inline std::string intToHuman(int64_t number)
  {
    const std::pair<int64_t, std::string> magnitudes[] = {
      std::make_pair(1e6, "M"),
      std::make_pair(1e3, "k")
    };
    for (auto m : magnitudes)
    {
      if (number > m.first)
        return std::to_string(number / m.first) + m.second;
    }
    return std::to_string(number);
  }

}  // namespace exploration
}  // namespace pal

#endif  // PAL_EXPLORATION_VISUALIZATION_
