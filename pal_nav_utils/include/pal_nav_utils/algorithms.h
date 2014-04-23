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
#include <vector>

#include <tf/transform_datatypes.h>

#include "pal_nav_utils/grid_map.h"

namespace pal
{

namespace nav
{

  static const int64_t UNREACHABLE = 1e12;

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


  typedef std::function<bool(const Grid&, index_t)> GridCellSelectorFunction;
  typedef std::function<int8_t(int8_t, int8_t)> ApplyMaskOperator;

  void apply_mask_at_idx(Grid& map, const GridMask& mask, ApplyMaskOperator op, index_t idx);
  void apply_mask_if(Grid& map, const GridMask& mask, ApplyMaskOperator op, GridCellSelectorFunction pred);

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

#endif  // PAL_NAV_UTILS_FRONTIERS_STRATEGY_H_
