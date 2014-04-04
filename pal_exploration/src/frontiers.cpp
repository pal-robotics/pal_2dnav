/**************************************************************************
**
**  frontiers.cpp
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 07-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#include <queue>

#include <pal_exploration/frontiers.h>

namespace pal
{

namespace exploration
{

  void Frontier::addCell(FrontierCell cell)
  {
    cells_.push_back(cell);
    orientation_ += (cell.orientation - orientation_) / cells_.size();
  }

  tf::Point Frontier::getCentroid() const
  {
    tf::Point centroid(0, 0, 0);
    for (const FrontierCell& cell : cells_)
    {
      centroid += cell.position;
    }
    return centroid / cells_.size();
  }

  // Returns the frontier cell nearest to the centroid (without path planning).
  index_t Frontier::getCentroidFrontierCell() const
  {
    // Returns the frontier cell nearest to the centroid of the cluster.
    tf::Point centroid = getCentroid();

    double bestDistance = -1;
    index_t bestIdx = -1;
    for (const FrontierCell& cell : cells_)
    {
      tf::Vector3 distanceVector = centroid - cell.position;
      double distance = distanceVector.length();
      if (bestDistance < 0 or distance < bestDistance)
      {
        bestDistance = distance;
        bestIdx = cell.idx;
      }
    }
    assert(bestDistance >= 0);
    return bestIdx;
  }

  index_t Frontier::getGoal(const GridMap &map, const DistanceMap& distmap) const
  {
    index_t centroid = getCentroidFrontierCell();

    /*
    RayTraceOp raytraceFunc = [](const GridMap& map, index_t idx) -> bool
    {
      return !map.isObstacle(idx);
    };
    */

    // If the centroid is unreachable, perform a BFS and
    // return the first cell found around it that is reachable.
    std::queue<std::pair<index_t, int>> queue;
    std::vector<bool> queued(map.size(), false);
    const int max_dist_in_meters = 2.0;
    const int max_depth = std::max(1.0, std::ceil(max_dist_in_meters / map.resolution));
    queue.push(std::make_pair(centroid, 0));
    while (!queue.empty())
    {
      index_t idx = queue.front().first;
      int depth = queue.front().second;
      queue.pop();

      // Is this a reachable cell from which we can see the centroid?
      if (isReachable(distmap[idx]) /*&& raytrace(map, idx, centroid, raytraceFunc)*/)
      {
        return idx;
      }
      if (depth < max_depth)
      {
        for (index_t nidx : map.getEightNeighbours(idx))
        {
          if (!queued[nidx] && !map.isDanger(nidx))
          {
            queue.push(std::make_pair(nidx, depth + 1));
            queued[nidx] = true;
          }
        }
      }
    }

    return -1;
  }

  bool Frontier::checkSize(int min_size) const
  {
    assert(cells_.size());
    int min_x = cells_.size();
    int min_y = cells_.size();
    int max_x = -1;
    int max_y = -1;
    for (const FrontierCell& cell : cells_)
    {
      min_x = std::min(min_x, cell.x);
      min_y = std::min(min_y, cell.y);
      max_x = std::max(max_x, cell.x);
      max_y = std::max(max_y, cell.y);
    }

    // TODO: make this actual area
    int max_side = std::max(max_x - min_x, max_y - min_y);
    return max_side >= min_size;
  }

}  // namespace exploration
}  // namespace pal
