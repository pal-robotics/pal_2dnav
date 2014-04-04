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

#ifndef PAL_EXPLORATION_FRONTIERS_STRATEGY_H_
#define PAL_EXPLORATION_FRONTIERS_STRATEGY_H_

#include <vector>
#include <memory>

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <pal_nav_utils/markers.h>
#include "pal_exploration/base_strategy.h"
#include "pal_exploration/frontiers.h"

namespace pal
{

namespace exploration
{

  class FrontiersStrategy : public BaseStrategy
  {

  public:
    FrontiersStrategy();
    ~FrontiersStrategy() {}

    void executeStep();
    void executeControlStep();
    void preempt();

  protected:
    std::vector<Frontier> findFrontiers(const GridMap& map) const;

    DistanceStruct calculateDistances() const;
    FrontiersPriorityQueue calculateFrontiers(const DistanceStruct& paths) const;

    void handleNoFrontiers();
    tf::Vector3 last_target_goal_;
    bool own_active_goal_;
    index_t num_target_goals_;  // number of goals sent to move_base

    void sendGoal(const geometry_msgs::Pose& pose);
    void reachedGoal(const actionlib::SimpleClientGoalState& state, const boost::shared_ptr<const move_base_msgs::MoveBaseResult>& result);

  private:
    ros::Publisher frontier_map_publisher_;
    ros::Publisher frontier_cloud_publisher_;

    ros::Publisher frontier_markers_publisher_;
    std::unique_ptr<MarkersCollection> frontier_markers_;

    ros::Publisher distance_cloud_publisher_;
    ros::Publisher distance_arrows_publisher_;
  };

}  // namespace exploration
}  // namespace pal

#endif  // PAL_EXPLORATION_FRONTIERS_STRATEGY_H_
