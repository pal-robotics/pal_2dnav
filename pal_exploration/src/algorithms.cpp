/**************************************************************************
**
**  algorithms.cpp
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 08-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#include <cstdint>
#include <queue>
#include <vector>
#include <utility>

#include <pal_exploration/algorithms.h>

namespace pal
{

namespace exploration
{

  DistanceStruct dijkstra(
    const GridMap& map, index_t targetIdx,
    IndicesFunction getEdges, CostFunction getCost)
  {
    std::vector<bool> visited(map.size(), false);
    DistanceMap distmap(map.size(), UNREACHABLE);
    PrevMap prevmap(map.size());

    typedef std::pair<int64_t, int> QueuedCell;
    std::priority_queue<QueuedCell, std::vector<QueuedCell>, std::greater<QueuedCell>> queue;

    distmap[targetIdx] = 0;
    queue.push(std::make_pair(0, targetIdx));

    while (!queue.empty())
    {
      index_t idx = queue.top().second;
      queue.pop();
      if (visited[idx]) continue;
      visited[idx] = true;
      const double dist = distmap[idx];
      for (index_t nidx : getEdges(idx))
      {
        ROS_ASSERT(nidx >= 0 && nidx < map.size());
        const double cand_dist = dist + getCost(idx, nidx);
        ROS_ASSERT(cand_dist >= dist);
        if (cand_dist < distmap[nidx])
        {
          distmap[nidx] = cand_dist;
          prevmap[nidx] = idx;
          queue.push(std::make_pair(cand_dist, nidx));
        }
      }
    }

    return DistanceStruct(targetIdx, distmap, prevmap);
  }

  void apply_mask_at_idx(GridMap& map, const GridMask& mask, ApplyMaskOperator op, index_t idx)
  {
    // Otherwise we can't calculate the center point
    ROS_ASSERT(mask.width % 2 == 1);
    ROS_ASSERT(mask.height % 2 == 1);

    int anchor_x = mask.width / 2;  // = std::ceil(max.width / 2) - 1
    int anchor_y = mask.height / 2;

    const int x = idx % map.width;
    const int y = idx / map.width;

    int cy = y - anchor_y;
    int cx;
    for (int i = 0; i < mask.height; ++i, ++cy)
    {
      if (cy < 0) continue;
      if (cy >= map.height) break;
      cx = x - anchor_x;

      for (int j = 0; j < mask.width; ++j, ++cx)
      {
        if (cx < 0) continue;
        if (cx >= map.width) break;

        const int cidx = cy * map.width + cx;
        const int mask_idx = i * mask.width + j;
        map.data[cidx] = op(map.data[cidx], mask.data[mask_idx]);
      }
    }
  }

  void apply_mask_if(GridMap& map, const GridMask& mask, ApplyMaskOperator op, GridCellSelectorFunction pred)
  {
    for (index_t idx = 0; idx < map.size(); ++idx)
    {
      if (pred(map, idx))
      {
        apply_mask_at_idx(map, mask, op, idx);
      }
    }
  }

}  // namespace exploration
}  // namespace pal
