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

#include <pal_nav_utils/grid_map.h>

#include <queue>
#include <utility>

#include "pal_nav_utils/algorithms.h"

namespace pal
{

namespace nav
{

  const int8_t GridMap::OBSTACLE = 100;
  const int8_t GridMap::DANGER = 60;
  const int8_t GridMap::UNREACHABLE = 50;
  const int8_t GridMap::UNKNOWN = -1;

  tf::Vector3 GridMap::getFrontierOrientation(index_t idx) const
  {
    const tf::Vector3 kRight(1, 0, 0);
    const tf::Vector3 kLeft(-1, 0, 0);
    const tf::Vector3 kUp(0, 1, 0);
    const tf::Vector3 kDown(0, -1, 0);

    int c = 0;
    tf::Vector3 v(0, 0, 0);
    auto add = [&c, &v](tf::Vector3 a) { v += a; ++c; };

    // 8-connectivity?
    if (idx - 1 >= 0 && data[idx - 1] == UNKNOWN) add(kLeft);
    if (idx + 1 < size() && data[idx + 1] == UNKNOWN) add(kRight);
    if (idx + width < size() && data[idx + width] == UNKNOWN) add(kUp);
    if (idx - width >= 0 && data[idx - width] == UNKNOWN) add(kDown);

    const int x = idx % width;
    const int y = idx / width;
    if (x == 0) add(kLeft);
    if (x == width - 1) add(kRight);
    if (y == 0) add(kUp);
    if (y == height - 1) add(kDown);

    ROS_ASSERT(c > 0);
    return v / c;
  }

  GridMask* makeSquareMask(int radius, int8_t value)
  {
    const int mask_w = (2 * radius + 1);
    const int mask_h = (2 * radius + 1);
    std::vector<int8_t> mask(mask_w * mask_h, value);
    return new GridMask(mask, mask_w, mask_h);
  }

  GridMask* makeCircularMask(int radius, int8_t value, int8_t default_value)
  {
    // The +1 is required so that the resulting size is odd (and
    // thus it has a center cell, required by apply_mask_if).
    const int mask_w = (2 * radius + 1);
    const int mask_h = (2 * radius + 1);
    std::vector<int8_t> mask(mask_w * mask_h, default_value);
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
    return new GridMask(mask, mask_w, mask_h);
  }

#define CHECK_RADIUS(r) if (radius == 0) return; ROS_ASSERT(radius > 0);

  void inflateObstacles(GridMap& map, int radius, int8_t value)
  {
    CHECK_RADIUS(radius);

    static GridMask* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask(radius, value);
      last_radius = radius;
    }

    ApplyMaskOperator op = [](int8_t cell, int8_t mask_cell)
    {
      return std::max(cell, mask_cell);
    };

    GridCellSelectorFunction pred = boost::bind(&GridMap::isObstacle, map, _2);

    apply_mask_if(map, *grid_mask, op, pred);
  }

  void freeRobotPose(GridMap& map, index_t idx, int radius)
  {
    CHECK_RADIUS(radius);

    static GridMask* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask(radius);
      last_radius = radius;
    }

    ApplyMaskOperator op = [](int8_t cell, int8_t mask_cell)
    {
      // Mark the cell as known & free
      return 0;
    };
    apply_mask_at_idx(map, *grid_mask, op, idx);
  }

  void inflateBlacklist(GridMap& map, std::set<tf::Vector3> positions, int radius)
  {
    CHECK_RADIUS(radius);

    static GridMask* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask(radius);
      last_radius = radius;
    }

    ApplyMaskOperator op = [](int8_t cell, int8_t mask_cell)
    {
      // If the cell isn't unknown, set it as unreachable.
      if (cell == GridMap::UNKNOWN)
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

  bool isRadiusFree(GridMap& map, index_t idx, int radius)
  {
    static GridMask* grid_mask = NULL;
    static int last_radius = -1;
    if (!grid_mask || radius != last_radius)
    {
      delete grid_mask;
      grid_mask = makeCircularMask(radius);
      last_radius = radius;
    }

    bool danger = false;
    ApplyMaskOperator op = [&danger](int8_t cell, int8_t mask_cell)
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

}  // namespace nav
}  // namespace pal
