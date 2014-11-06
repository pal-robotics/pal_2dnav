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

#ifndef PAL_EXPLORATION_FRONTIERS_
#define PAL_EXPLORATION_FRONTIERS_

#include <vector>
#include <utility>
#include <queue>

#include <tf/transform_datatypes.h>

#include <pal_nav_utils/grid_map.h>
#include <pal_nav_utils/algorithms.h>

namespace pal
{

namespace exploration
{

  class FrontierCell
  {
  public:
    FrontierCell(const nav::GridMap<int8_t>& map, nav::index_t idx)
      : idx(idx),
        x(idx % map.width),
        y(idx / map.width),
        position(map.getPositionFromIdx(idx)),
        orientation(map.getFrontierOrientation(idx)) {}

    ~FrontierCell() {}

    nav::index_t idx;
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

    nav::index_t getCentroidFrontierCell() const;

    // Returns a reachable goal position.
    nav::index_t getGoal(const nav::GridMap<int8_t>& map, const nav::DistanceMap& distmap) const;

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
