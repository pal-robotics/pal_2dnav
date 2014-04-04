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

#ifndef PAL_NAV_UTILS_GRID_MAP_
#define PAL_NAV_UTILS_GRID_MAP_

#include <cstdint>
#include <cmath>
#include <vector>
#include <stack>

#include <ros/ros.h>
#include <ros/assert.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pal_nav_utils/grid.h"

namespace pal
{

namespace nav
{

  class GridMap : public Grid
  {
  public:

    GridMap(const nav_msgs::OccupancyGrid& map)
      : Grid(map.data, map.info.width, map.info.height),
        resolution(map.info.resolution), origin(map.info.origin)
    {
    }

    ~GridMap() {}

    inline index_t getIdxForPosition(const tf::Point& point) const
    {
      int x = (point.x() - origin.position.x) / resolution;
      int y = (point.y() - origin.position.y) / resolution;
      return getIdx(x, y);
    }

    inline index_t getIdxForPosition(const geometry_msgs::Point& point) const
    {
      return getIdxForPosition(tf::Point(point.x, point.y, 0));
    }

    inline tf::Vector3 getPositionFromIdx(index_t idx) const
    {
      double x = (idx % width) * resolution + origin.position.x;
      double y = (idx / width) * resolution + origin.position.y;
      return tf::Vector3(x, y, 0);
    }

    inline double xToCoord(int x) const
    {
      return x * resolution + origin.position.x;
    }

    inline double yToCoord(int y) const
    {
      return y * resolution + origin.position.y;
    }

    inline double euclideanDistance(index_t idx1, index_t idx2) const
    {
      const int x1 = idx1 / width;
      const int y1 = idx1 % width;

      const int x2 = idx2 / width;
      const int y2 = idx2 % width;

      return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) * resolution;
    }

    inline bool isObstacle(index_t idx) const {
      ROS_ASSERT(idx >= 0 && idx < size());
      return data[idx] >= OBSTACLE;
    }

    inline bool isDanger(index_t idx) const {
      ROS_ASSERT(idx >= 0 && idx < size());
      return data[idx] >= DANGER;
    }

    inline bool isUnknown(index_t idx) const {
      return data[idx] == UNKNOWN;
    }

    /*
     * Returns true iff `idx' is an explored cell that:
     *  - is surrounded by at least one unknown cell, or
     *  - is at the map boundary.
     */
    inline bool isFrontier(index_t idx) const {
      if (data[idx] == UNKNOWN) return false;

      // FIXME change to 8-neighbours?
      for (index_t nidx : getFourNeighbours(idx))
      {
        if (isUnknown(nidx)) return true;
      }

      const int x = idx % width;
      const int y = idx / width;
      const bool is_on_map_edge = x == 0 || x == width - 1 || y == 0 || y == height - 1;
      if (is_on_map_edge) return true;

      return false;
    }

    tf::Vector3 getFrontierOrientation(index_t idx) const;

    /**
     * @brief Checks whether the cells in a square space are known.
     * @param point Coordinates for the center of the box.
     * @param size Size of the edges (in meters).
     * @return 1 if it fits, 0 it if doesn't, -1 if it'd end up outside the map
     */
    inline int isBoxKnown(const geometry_msgs::Point& point, double size) const
    {
      const int R = std::ceil(size / resolution);
      int cx = (point.x - origin.position.x) / resolution;
      int cy = (point.y - origin.position.y) / resolution;

      const int r = std::floor(R / 2);
      for (int x = cx - r; x <= cx + r; ++x)
      {
        if (x < 0 || x >= width)
        {
          return -1;
        }
        for (int y = cy - r; y <= cy + r; ++y)
        {
          if (y < 0 || y >= height)
          {
            return -1;
          }
          if (data[getIdx(x, y)] == UNKNOWN)
            return 0;
        }
      }
      return 1;
    }

    static const int8_t OBSTACLE;
    static const int8_t DANGER;
    static const int8_t UNREACHABLE;
    static const int8_t UNKNOWN;

    double resolution;
    geometry_msgs::Pose origin;
  };

  void inflateObstacles(GridMap& map, int radius);
  void freeRobotPose(GridMap& map, index_t idx, int radius);
  void inflateBlacklist(GridMap& map, std::set<tf::Vector3> positions, int radius);

  bool isRadiusFree(GridMap& map, index_t idx, int radius);

  GridMask* makeSquareMask(int radius);
  GridMask* makeCircularMask(int radius, int8_t value=GridMap::DANGER, int8_t default_value=GridMap::UNKNOWN);

}  // namespace nav
}  // namespace pal

#endif  // PAL_NAV_UTILS_GRID_MAP_
