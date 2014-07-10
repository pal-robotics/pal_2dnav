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

#ifndef PAL_NAV_UTILS_ALGORITHMS_H_
#define PAL_NAV_UTILS_ALGORITHMS_H_

#include <functional>
#include <utility>
#include <vector>
#include <queue>
#include <limits>

#include <tf/transform_datatypes.h>

#include <pal_nav_utils/grid_map.h>

namespace pal
{

namespace nav
{

  static const int64_t UNREACHABLE = std::numeric_limits<int64_t>::max();

  typedef std::function<indices_t(index_t)> IndicesFunction;
  typedef std::function<double(index_t, index_t)> CostFunction;

  typedef std::vector<double> DistanceMap;
  typedef std::vector<index_t> PrevMap;

  struct DistanceStruct {
    DistanceStruct(index_t target, DistanceMap distance, PrevMap prevmap)
      : target(target), distance(distance), prevmap(prevmap) {}

    index_t target;
    DistanceMap distance;
    PrevMap prevmap;
  };

  DistanceStruct dijkstra(
      const GridMap& map, index_t targetIdx,
      IndicesFunction getEdges, CostFunction getCost);

  template <typename T>
  struct GridCellSelectorFunction
  {
    typedef std::function<bool(const Grid<T>&, index_t)> type;
  };
  typedef std::function<int8_t(int8_t, int8_t)> ApplyMaskOperator;

  template <typename T>
  void apply_mask_at_idx(Grid<T>& map, const typename GridMask<T>::type& mask, ApplyMaskOperator op, index_t idx)
  {
    // Otherwise we can't calculate the center point
    ROS_ASSERT(mask.width % 2 == 1);
    ROS_ASSERT(mask.height % 2 == 1);

    int anchor_x = mask.width / 2;  // = std::ceil(max.width / 2) - 1
    int anchor_y = mask.height / 2;

    const int x = idx % map.width;
    const int y = idx / map.width;

    // Calculate y range (checking mask and map boundaries)
    int start_cy = y - anchor_y;
    int start_i = 0;
    if (start_cy < 0)
    {
      start_i = -start_cy;
      start_cy = 0;
    }
    int height = std::min(mask.height - start_i, map.height - start_cy);

    // Calculate x range (checking mask and map boundaries)
    int start_cx = x - anchor_x;
    int start_j = 0;
    if (start_cx < 0)
    {
      start_j = -start_cx;
      start_cx = 0;
    }
    int width = std::min(mask.width - start_j, map.width - start_cx);

    for (int i = start_i, cy = start_cy; i < height; ++i, ++cy)
    {
      for (int j = start_j, cx = start_cx; j < width; ++j, ++cx)
      {
        const int cidx = cy * map.width + cx;
        const int mask_idx = i * mask.width + j;
        map.data[cidx] = op(map.data[cidx], mask.data[mask_idx]);
      }
    }
  }

  template <typename T>
  void apply_mask_if(Grid<T>& map, const typename GridMask<T>::type& mask, ApplyMaskOperator op, typename GridCellSelectorFunction<T>::type pred)
  {
    for (index_t idx = 0; idx < map.size(); ++idx)
    {
      if (pred(map, idx))
      {
        apply_mask_at_idx(map, mask, op, idx);
      }
    }
  }

  inline int8_t MaskOperatorMax(int8_t cell, int8_t mask_cell)
  {
    return std::max(cell, mask_cell);
  }

  inline bool isReachable(double distance)
  {
    return distance < UNREACHABLE - 0.5;
  }

}  // namespace nav
}  // namespace pal

#endif  // PAL_NAV_UTILS_ALGORITHMS_H_
