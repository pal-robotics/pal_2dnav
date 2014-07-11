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
#include <utility>

#include <ros/ros.h>
#include <ros/assert.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pal_nav_utils/grid.h>
#include <pal_nav_utils/algorithms.h>

namespace pal
{

namespace nav
{

  template <typename T>
  class GridMap : public Grid<T>
  {
  public:

    typedef typename Grid<T>::value_type value_type;

    GridMap(int width, int height, double resolution,
        const geometry_msgs::Pose& origin)
      : Grid<T>(width, height),
        resolution(resolution), origin(origin)
    {
    }

    GridMap(const nav_msgs::OccupancyGrid& map)
      : Grid<T>(map.data, map.info.width, map.info.height),
        resolution(map.info.resolution), origin(map.info.origin)
    {
    }

    ~GridMap() {}

    inline index_t getIdxForPosition(const tf::Point& point) const
    {
      int x = coordToX(point.x());
      int y = coordToY(point.y());
      return this->getIdx(x, y);
    }

    inline index_t getIdxForPosition(const geometry_msgs::Point& point) const
    {
      return getIdxForPosition(tf::Point(point.x, point.y, 0));
    }

    inline tf::Vector3 getPositionFromIdx(index_t idx) const
    {
      double x = xToCoord(idx % this->width);
      double y = yToCoord(idx / this->width);
      return tf::Vector3(x, y, 0);
    }

    inline int coordToX(double x) const
    {
      return (x - origin.position.x) / resolution;
    }

    inline int coordToY(double y) const
    {
      return (y - origin.position.y) / resolution;
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
      const int x1 = idx1 / this->width;
      const int y1 = idx1 % this->width;

      const int x2 = idx2 / this->width;
      const int y2 = idx2 % this->width;

      return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) * resolution;
    }

    inline bool isObstacle(index_t idx) const {
      ROS_ASSERT(idx >= 0 && idx < this->size());
      return this->data[idx] >= OBSTACLE;
    }

    inline bool isDanger(index_t idx) const {
      ROS_ASSERT(idx >= 0 && idx < this->size());
      return this->data[idx] >= DANGER;
    }

    inline bool isUnknown(index_t idx) const {
      return this->data[idx] == UNKNOWN;
    }

    /*
     * Returns true iff `idx' is an explored cell that:
     *  - is surrounded by at least one unknown cell, or
     *  - is at the map boundary.
     */
    inline bool isFrontier(index_t idx) const {
      if (this->data[idx] == UNKNOWN) return false;

      // FIXME change to 8-neighbours?
      for (index_t nidx : this->getFourNeighbours(idx))
      {
        if (isUnknown(nidx)) return true;
      }

      const int x = idx % this->width;
      const int y = idx / this->width;
      const bool is_on_map_edge = x == 0 || x == this->width - 1 ||
                                  y == 0 || y == this->height - 1;

      return is_on_map_edge;
    }

    tf::Vector3 getFrontierOrientation(index_t idx) const
    {
      const tf::Vector3 kRight(1, 0, 0);
      const tf::Vector3 kLeft(-1, 0, 0);
      const tf::Vector3 kUp(0, 1, 0);
      const tf::Vector3 kDown(0, -1, 0);

      int c = 0;
      tf::Vector3 v(0, 0, 0);
      auto add = [&c, &v](tf::Vector3 a) { v += a; ++c; };

      // 8-connectivity?
      if (idx - 1 >= 0 &&
          this->data[idx - 1] == UNKNOWN) add(kLeft);
      if (idx + 1 < this->size() &&
          this->data[idx + 1] == UNKNOWN) add(kRight);
      if (idx + this->width < this->size() &&
          this->data[idx + this->width] == UNKNOWN) add(kUp);
      if (idx - this->width >= 0 &&
          this->data[idx - this->width] == UNKNOWN) add(kDown);

      const int x = idx % this->width;
      const int y = idx / this->width;
      if (x == 0) add(kLeft);
      if (x == this->width - 1) add(kRight);
      if (y == 0) add(kUp);
      if (y == this->height - 1) add(kDown);

      ROS_ASSERT(c > 0);
      return v / c;
    }

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
        if (x < 0 || x >= this->width)
        {
          return -1;
        }
        for (int y = cy - r; y <= cy + r; ++y)
        {
          if (y < 0 || y >= this->height)
          {
            return -1;
          }
          if (this->data[this->getIdx(x, y)] == UNKNOWN)
            return 0;
        }
      }
      return 1;
    }

    static constexpr value_type OBSTACLE = 100;
    static constexpr value_type DANGER = 60;
    static constexpr value_type UNREACHABLE = 50;
    static constexpr value_type UNKNOWN = -1;

    double resolution;
    geometry_msgs::Pose origin;
  };

#define CHECK_RADIUS(r) if (radius == 0) return; ROS_ASSERT(radius > 0);

  template <typename T>
  typename GridMask<T>::type* makeCircularMask(int radius,
      typename GridMap<T>::value_type value = GridMap<T>::DANGER,
      typename GridMap<T>::value_type default_value = GridMap<T>::UNKNOWN)
  {
    typedef typename GridMap<T>::value_type value_type;

    // The +1 is required so that the resulting size is odd (and
    // thus it has a center cell, required by apply_mask_if).
    const int mask_w = (2 * radius + 1);
    const int mask_h = (2 * radius + 1);
    std::vector<value_type> mask(mask_w * mask_h, default_value);
    for (int y = 0; y < mask_h; ++y)
    {
      for (int x = 0; x < mask_w; ++x)
      {
        if (pow(x - radius, 2) + pow(y - radius, 2) <= pow(radius, 2))
        {
          mask[x + y * mask_w] = value;
        }
      }
    }
    return new typename GridMask<T>::type(mask, mask_w, mask_h);
  }

  template <typename T>
  void inflateObstacles(GridMap<T>& map, int radius,
      typename GridMap<T>::value_type value = GridMap<T>::DANGER)
  {
    CHECK_RADIUS(radius);

    typedef typename GridMap<T>::value_type value_type;

    static typename GridMask<value_type>::type* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask<value_type>(radius, value);
      last_radius = radius;
    }

    typename ApplyMaskOperator<T>::type op = [](value_type cell, value_type mask_cell)
    {
      return std::max(cell, mask_cell);
    };

    typename GridCellSelectorFunction<value_type>::type pred =
      boost::bind(&GridMap<T>::isObstacle, map, _2);

    apply_mask_if(map, *grid_mask, op, pred);
  }

  template <typename T>
  void freeRobotPose(GridMap<T>& map, index_t idx, int radius)
  {
    CHECK_RADIUS(radius);

    typedef typename GridMap<T>::value_type value_type;

    static typename GridMask<value_type>::type* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask<value_type>(radius);
      last_radius = radius;
    }

    typename ApplyMaskOperator<T>::type op =
      [](value_type cell, value_type mask_cell)
    {
      // Mark the cell as known & free
      return 0;
    };
    apply_mask_at_idx(map, *grid_mask, op, idx);
  }

  template <typename T>
  void inflateBlacklist(GridMap<T>& map,
      std::set<tf::Vector3> positions, int radius)
  {
    CHECK_RADIUS(radius);

    typedef typename GridMap<T>::value_type value_type;

    static typename GridMask<value_type>::type* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask<value_type>(radius);
      last_radius = radius;
    }

    typename ApplyMaskOperator<T>::type op =
      [](value_type cell, value_type mask_cell) -> value_type
    {
      // If the cell isn't unknown, set it as unreachable.
      if (cell == GridMap<T>::UNKNOWN)
        return cell;
      else
        return std::max(cell, mask_cell);
    };

    for (tf::Vector3 pos : positions)
    {
      index_t idx = map.getIdxForPosition(pos);
      ROS_WARN_STREAM("Applying mask at idx=" << idx);
      apply_mask_at_idx(map, *grid_mask, op, idx);
    }
  }

  template <typename T>
  bool isRadiusFree(GridMap<T>& map, index_t idx, int radius)
  {
    typedef typename GridMap<T>::value_type value_type;

    static typename GridMask<value_type>::type* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask<value_type>(radius);
      last_radius = radius;
    }

    bool danger = false;
    typename ApplyMaskOperator<T>::type op =
      [&danger](value_type cell, value_type mask_cell)
    {
      if (mask_cell && cell > 0)
      {
        danger = true;
      }
      return cell;
    };
    apply_mask_at_idx(map, *grid_mask, op, idx);

    return !danger;
  }

  template <typename T>
  typename GridMask<T>::type* makeSquareMask(int radius,
      typename GridMap<T>::value_type value = GridMap<T>::DANGER)
  {
    typedef typename GridMap<T>::value_type value_type;

    const int mask_w = (2 * radius + 1);
    const int mask_h = (2 * radius + 1);
    std::vector<value_type> mask(mask_w * mask_h, value);
    return new typename GridMask<T>::type(mask, mask_w, mask_h);
  }

}  // namespace nav
}  // namespace pal

#endif  // PAL_NAV_UTILS_GRID_MAP_
