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

#ifndef PAL_EXPLORATION_BASE_STRATEGY_H_
#define PAL_EXPLORATION_BASE_STRATEGY_H_

#include <atomic>
#include <memory>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include <pal_nav_utils/grid_map.h>
#include "pal_exploration/PalExplorationConfig.h"

namespace pal
{

namespace exploration
{

  class BaseStrategy
  {

  public:
    BaseStrategy();
    virtual ~BaseStrategy() {}

    // Blocking call.
    bool execute(std::function<bool()> check);

    // Implements the exploration logic. Called periodically by execute(),
    // if the SLAM map changed.
    virtual void executeStep() = 0;

    // Implements the exploration logic. Called periodically by execute(),
    // when the SLAM map didn't change.
    virtual void executeControlStep() = 0;

    // Called periodically by execute() while there isn't a SLAM map.
    virtual void executeInitStep(bool updated_map);

    virtual void preempt() {}

    virtual void loopClosureStart() {}

    virtual void loopClosureEnd()
    {
      blacklist_.clear();
    }

    void setPreempted() {
      // Actually the atomic<> is not needed (since we only have one
      // thread, but this way it is future-proof).
      preempted_.store(true);
      preempt();
    }

    void configCb(pal_exploration::PalExplorationConfig& config, uint32_t level);

  protected:
    bool isPreempted() const {
      return preempted_.load();
    }

    ros::CallbackQueue callback_queue_;
    ros::NodeHandle node_;
    ros::Subscriber slam_map_subscriber_;
    ros::Subscriber slam_pose_subscriber_;
    ros::Subscriber move_base_subscriber_;

    ros::Subscriber loop_closing_subscriber_;
    ros::Subscriber loop_closed_subscriber_;

    std::atomic<bool> preempted_;
    bool finished_;

    std::vector<int8_t> original_map_;
    std::unique_ptr<nav::GridMap<int8_t>> map_;
    geometry_msgs::Pose pose_;
    bool pose_initialized_;
    bool map_ready_;
    bool map_updated_;

    bool move_base_is_active_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    ros::Publisher cmd_vel_publisher_;

    void slamMapCb(const nav_msgs::OccupancyGrid&);
    void slamPoseCb(const geometry_msgs::PoseWithCovarianceStamped&);
    void moveBaseCb(const actionlib_msgs::GoalStatusArray&);

    void loopClosingCb(const std_msgs::Bool&);
    void loopClosedCb(const std_msgs::Bool&);

    // Subclasses can put indices into this. With the default implementation,
    // the list will be reset every time loopClosureEnd is called.
    std::set<tf::Vector3> blacklist_;

    void addToBlacklist(tf::Vector3 pos);

    // Configuration parameters
    double safety_radius_;
    double blacklist_radius_;
    double neglect_area_;
    double initial_free_radius_;
    double update_distance_;
    double goal_edge_margin_;

  private:
    ros::Publisher obstacle_map_publisher_;
  };

}  // namespace exploration
}  // namespace pal

#endif  // PAL_EXPLORATION_BASE_STRATEGY_H_
