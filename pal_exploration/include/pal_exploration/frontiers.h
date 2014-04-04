/**************************************************************************
**
**  frontiers.h
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 07-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef PAL_EXPLORATION_FRONTIERS_
#define PAL_EXPLORATION_FRONTIERS_

#include <vector>
#include <utility>
#include <queue>

#include <tf/transform_datatypes.h>

#include <pal_exploration/grid_map.h>
#include <pal_exploration/algorithms.h>

namespace pal
{

namespace exploration
{

  class FrontierCell
  {
  public:
    FrontierCell(const GridMap& map, index_t idx)
      : idx(idx),
        x(idx % map.width),
        y(idx / map.width),
        position(map.getPositionFromIdx(idx)),
        orientation(map.getFrontierOrientation(idx)) {}

    ~FrontierCell() {}

    index_t idx;
    int x;
    int y;
    tf::Vector3 position;
    tf::Vector3 orientation;
  };

  class Frontier
  {
  public:
    Frontier() : pending_(0), orientation_(0, 0, 0) {}
    ~Frontier() {}

    void addCell(FrontierCell cell);

    inline const std::vector<FrontierCell>& cells(void) const
    {
      return cells_;
    }

    inline const tf::Vector3& getOrientation() const
    {
      ROS_ASSERT(!cells_.empty());
      return orientation_;
    }

    inline tf::Vector3 getOrientation(const tf::Point& position) const
    {
      ROS_ASSERT(!cells_.empty());
      tf::Point vec = getCentroid() - position;
      vec.normalize();
      return vec;
    }

    tf::Point getCentroid() const;

    index_t getCentroidFrontierCell() const;

    // Returns a reachable goal position.
    index_t getGoal(const GridMap& map, const DistanceMap& distmap) const;

    inline bool operator< (const Frontier& rhs) const
    {
      return cells_.size() < rhs.cells_.size();
    }

    inline int size() const
    {
      return cells_.size();
    }

    bool checkSize(int min_size) const;

  private:
    std::vector<FrontierCell> cells_;
    int pending_;
    tf::Vector3 orientation_;
  };

  typedef std::priority_queue<std::pair<double, Frontier>> FrontiersPriorityQueue;

}  // namespace exploration
}  // namespace pal

#endif  // PAL_EXPLORATION_DATATYPES_FRONTIERS_
