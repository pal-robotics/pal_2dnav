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

#include "pal_exploration/base_strategy.h"

#include <boost/bind.hpp>
#include <queue>
#include <utility>

#include <pal_nav_utils/visualization.h>
#include "pal_exploration/visualization.h"

namespace pal
{

namespace exploration
{

  BaseStrategy::BaseStrategy()
    : node_(), preempted_(false), finished_(false),
      pose_initialized_(false), map_ready_(false), map_updated_(false),
      move_base_is_active_(false),
      move_base_client_("move_base")
  {
    ROS_INFO_STREAM("Starting Exploration...");

    node_.setCallbackQueue(&callback_queue_);

    ros::NodeHandle private_node("~");
    private_node.setCallbackQueue(&callback_queue_);

    slam_map_subscriber_ = node_.subscribe("/map", 1, &BaseStrategy::slamMapCb, this);
    slam_pose_subscriber_ = node_.subscribe("/slam_karto_pose", 1, &BaseStrategy::slamPoseCb, this);

    move_base_subscriber_ = node_.subscribe("/move_base/status", 1, &BaseStrategy::moveBaseCb, this);

    loop_closing_subscriber_ = node_.subscribe("/stop_closing_loop", 1, &BaseStrategy::loopClosingCb, this);
    loop_closed_subscriber_ = node_.subscribe("/slam_karto/loop_closed", 1, &BaseStrategy::loopClosedCb, this);

    obstacle_map_publisher_ = private_node.advertise<nav_msgs::OccupancyGrid>("obstacle_map", 1, true);
    cmd_vel_publisher_ = node_.advertise<geometry_msgs::Twist>("/nav_vel", 1, true);
  }

  void BaseStrategy::configCb(pal_exploration::PalExplorationConfig& config, uint32_t level)
  {
    ROS_INFO("Configuration updated. Some changes may not take effect until the map is updated.");
    safety_radius_ = config.safety_radius;
    blacklist_radius_ = config.blacklist_radius;
    neglect_area_= config.neglect_area;
    initial_free_radius_ = config.initial_free_radius;
    update_distance_ = config.update_distance;
    goal_edge_margin_ = config.goal_edge_margin;
  }

  void BaseStrategy::slamMapCb(const nav_msgs::OccupancyGrid& slam_map)
  {
    if (finished_)
      return;

    ros::WallTime startTime = ros::WallTime::now();
    ROS_INFO_STREAM("Received new map. Processing data... (safety radius = " << safety_radius_ << ")");
    GridMap* map = new GridMap(slam_map);

    const int Rinflate = std::ceil(safety_radius_ / map->resolution);
    inflateObstacles(*map, Rinflate);

    const int Rblacklist = std::ceil(blacklist_radius_ / map->resolution);
    inflateBlacklist(*map, blacklist_, Rblacklist);

    ros::WallDuration elapsedTime = ros::WallTime::now() - startTime;
    ROS_INFO_STREAM("Map replaced. Tweaked bits for " << elapsedTime.toSec() << "s.");
    map_.reset(map);
    map_updated_ = true;
  }

  void BaseStrategy::addToBlacklist(tf::Vector3 pos)
  {
    blacklist_.insert(pos);

    // Update existing map.
    const int Rblacklist = std::ceil(blacklist_radius_ / map_->resolution);
    std::set<tf::Vector3> single;
    single.insert(pos);
    inflateBlacklist(*map_, single, Rblacklist);
  }

  void BaseStrategy::slamPoseCb(const geometry_msgs::PoseWithCovarianceStamped& slam_pose)
  {
    ROS_ASSERT(slam_pose.header.frame_id == "map");
    pose_initialized_ = true;

    pose_ = slam_pose.pose.pose;
  }

  void BaseStrategy::moveBaseCb(const actionlib_msgs::GoalStatusArray& status_arr)
  {
    bool is_active = false;
    for (actionlib_msgs::GoalStatus status : status_arr.status_list)
    {
      if (status.status == actionlib_msgs::GoalStatus::ACTIVE)
      {
        is_active = true;
        break;
      }
    }
    move_base_is_active_ = is_active;
  }

  void BaseStrategy::loopClosingCb(const std_msgs::Bool&)
  {
    loopClosureStart();
  }

  void BaseStrategy::loopClosedCb(const std_msgs::Bool&)
  {
    loopClosureEnd();
  }

  // The given `check' function must be called every iteration.
  // It may call setPreempted() or perform other tasks.
  bool BaseStrategy::execute(std::function<bool()> check)
  {
    ros::Rate r(2);  // Hz
    while (node_.ok() && check() && !isPreempted() && !finished_)
    {
      callback_queue_.callAvailable();

      // TODO: check map age.
      // TODO: getNumPublishers here sometimes segfaults on
      //       succeeded/preempted, for no apparent reason.
      if (slam_map_subscriber_.getNumPublishers() == 0)
      {
        // SLAM is not running.
        ROS_INFO("Waiting for SLAM to start publishing a map...");
      }
      else if (map_ && pose_initialized_)
      {
        if (obstacle_map_publisher_.getNumSubscribers() > 0)
        {
          obstacle_map_publisher_.publish(createOccupancyGrid(*map_));
        }

        if (!map_ready_)
        {
          const int Rfree = std::ceil(initial_free_radius_ / map_->resolution);
          freeRobotPose(*map_, map_->getIdxForPosition(pose_.position), Rfree);

          // TODO: This is too conservative. "!= -1" should be used instead, but
          //       for now it's like this to handle the point of a map where the
          //       only frontier is the robot position.
          if (map_updated_ && map_->isBoxKnown(pose_.position, safety_radius_) == 1)
          {
            map_ready_ = true;
          }
          else
          {
            ROS_INFO_COND(map_updated_, "Position is outside map. Initialization manoeuvres...");
            executeInitStep(map_updated_);
          }
        }

        if (map_ready_)
        {
          if (map_updated_)
          {
            executeStep();
          }
          else
          {
            executeControlStep();
          }
        }

        map_updated_ = false;
      }
      else
      {
        if (map_updated_)
        {
          ROS_INFO("Received map is empty. Initialization manoeuvres...");
        }
        executeInitStep(map_updated_);
      }

      r.sleep();
    }

    return finished_;
  }

  void BaseStrategy::executeInitStep(bool updated_map)
  {
    if (pose_initialized_)
    {
      ROS_INFO("Executing initialization movement...");

      // Rotate robot for 0.5 seconds.
      geometry_msgs::Twist twist;
      twist.angular.z = 0.5;
      cmd_vel_publisher_.publish(twist);
      ros::Duration(1.0).sleep();
      cmd_vel_publisher_.publish(geometry_msgs::Twist());
    }
    else
    {
      ROS_INFO("Position not available - aborted.");
    }
  }

}  // namespace exploration
}  // namespace pal
