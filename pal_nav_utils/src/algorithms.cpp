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

#include "pal_nav_utils/algorithms.h"

#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

namespace pal
{

namespace nav
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

}  // namespace nav
}  // namespace pal
