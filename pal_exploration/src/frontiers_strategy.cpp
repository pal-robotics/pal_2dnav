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

#include <boost/bind.hpp>
#include <cmath>
#include <string>
#include <queue>
#include <utility>
#include <cstdlib>
#include <ctime>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <pal_nav_utils/visualization.h>
#include <pal_nav_utils/algorithms.h>
#include "pal_exploration/frontiers_strategy.h"
#include "pal_exploration/visualization.h"

namespace pal
{

namespace exploration
{

  FrontiersStrategy::FrontiersStrategy()
    : BaseStrategy(),
      own_active_goal_(false), num_target_goals_(0),
      frontier_markers_(nullptr)
  {
    ros::NodeHandle private_node("~");

    frontier_map_publisher_ = private_node.advertise<nav_msgs::OccupancyGrid>("frontier_map", 1, true);
    frontier_cloud_publisher_ = private_node.advertise<sensor_msgs::PointCloud2>("frontier_cloud", 1, true);

    frontier_markers_publisher_ = private_node.advertise<visualization_msgs::MarkerArray>("frontier_marker_array", 1, true);
    frontier_markers_.reset(new MarkersCollection(frontier_markers_publisher_));
    frontier_markers_->addNamespace("frontier_size", 1.0, 0, 0);
    frontier_markers_->addNamespace("frontier_distance", 0, 0.5, 0);

    distance_cloud_publisher_ = private_node.advertise<sensor_msgs::PointCloud2>("distance_cloud", 1, true);
    distance_arrows_publisher_ = private_node.advertise<geometry_msgs::PoseArray>("distance_arrows", 1, true);
  }

  void FrontiersStrategy::executeStep()
  {
    DistanceStruct paths = calculateDistances();
    DistanceMap distmap = paths.distance;

    FrontiersPriorityQueue frontiers = calculateFrontiers(paths);

    if (move_base_is_active_ && !own_active_goal_)
    {
      ROS_INFO("Withholding goal; external entity is moving the robot.");
      return;
    }

    // Move to best frontier
    if (frontiers.empty())
    {
      handleNoFrontiers();
      return;
    }
    double score = frontiers.top().first;
    const Frontier& frontier = frontiers.top().second;

    index_t target_goal = frontier.getGoal(*map_, distmap);

    double distToPreviousPoint = UNREACHABLE;
    if (own_active_goal_)
    {
      // TODO: actually check that there is a valid path between them
      index_t active_idx = map_->getIdxForPosition(last_target_goal_);
      distToPreviousPoint = map_->euclideanDistance(target_goal, active_idx); // returns meters
    }

    if (distToPreviousPoint < map_->resolution)
    {
      // Same goal, do nothing.
    }
    else if (distToPreviousPoint < update_distance_  && (distmap[target_goal] * map_->resolution) > update_distance_)
    {
      ROS_INFO("New goal is close to current goal; not changing it for now.");
    }
    else
    {
      geometry_msgs::Pose pose;
      tf::Vector3 v = map_->getPositionFromIdx(target_goal);
      pose.position.x = v.x();
      pose.position.y = v.y();
      v = frontier.getOrientation(v);
      double orientation = std::atan2(v.y(), v.x());
      pose.orientation.w = std::cos(orientation / 2.0);
      pose.orientation.z = std::sin(orientation / 2.0);

      sendGoal(pose);
      ROS_INFO_STREAM("New goal sent (target frontier has score " << score << ").");
    }
  }

  // Unchanged map
  void FrontiersStrategy::executeControlStep()
  {
    if (!move_base_is_active_)
    {
      executeStep();
    }
  }

  void FrontiersStrategy::sendGoal(const geometry_msgs::Pose& pose)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose = pose;
    move_base_client_.sendGoal(goal, boost::bind(&FrontiersStrategy::reachedGoal, this, _1, _2));

    last_target_goal_ = tf::Vector3(pose.position.x, pose.position.y, 0);
    own_active_goal_ = true;
    num_target_goals_++;
  }

  void FrontiersStrategy::handleNoFrontiers()
  {
    // Is the robot stuck somewhere (according to our inflated map)?
    const index_t idx = map_->getIdxForPosition(pose_.position);
    const int R = std::ceil(safety_radius_ / map_->resolution);
    if (!isRadiusFree(*map_, idx, R))
    {
      if (own_active_goal_)
      {
        // Just wait for move_base to move somewhere else (or to give up).
        ROS_WARN("Can't calculate distmap from current position...");
      }
      else
      {
        ROS_WARN("Robot stuck in obstacles. Attempting to return to start position...");
        geometry_msgs::Pose originPose;
        originPose.position.x = 0;
        originPose.position.y = 0;
        originPose.orientation.w = 1;
        sendGoal(originPose);
      }
      return;
    }

    // Stop moving to previous goal.
    if (own_active_goal_)
    {
      move_base_client_.cancelGoal();
      own_active_goal_ = false;
    }

    // Decide what is going on.
    if (num_target_goals_ > 0)
    {
      ROS_INFO("Finished exploration!");
      finished_ = true;
    }
    else
    {
      ROS_WARN("No frontiers available! This should not happen.");
    }
  }

  void FrontiersStrategy::reachedGoal(const actionlib::SimpleClientGoalState& state, const boost::shared_ptr<const move_base_msgs::MoveBaseResult>& result)
  {
    if (finished_)
      return;

    // Not moving anymore.
    own_active_goal_ = false;

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // OK!
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_WARN("Goal could not be reached. Adding to blacklist.");
      addToBlacklist(last_target_goal_);
    }
    else
    {
      // Preempted or other problem. We'll just retry.
      ROS_WARN_STREAM("Didn't complete goal, state: " << state.toString());
    }
  }

  DistanceStruct FrontiersStrategy::calculateDistances() const
  {
    index_t startIdx = map_->getIdxForPosition(pose_.position);
    auto getEdges = boost::bind(&GridMap::getFourNeighbours, *map_, _1);
    const int sc = std::ceil(goal_edge_margin_ / map_->resolution);
    auto getCost = [&map_, &sc](index_t idx, index_t nidx) -> double
    {
      if (map_->data[nidx] != 0) return UNREACHABLE;

      // move_base doesn't support moving to the very edge of the map
      int x = nidx % map_->width;
      int y = nidx / map_->width;
      if (x < sc || x + sc >= map_->width || y < sc || y + sc >= map_->height)
          return UNREACHABLE;

      return 1.0;
    };
    DistanceStruct paths = dijkstra(*map_, startIdx, getEdges, getCost);
    DistanceMap distmap = paths.distance;

    if (distance_cloud_publisher_.getNumSubscribers() > 0)
    {
      std::vector<int8_t> tmpv(distmap.size());
      const int64_t maxVal = 200;
      for (uint i = 0; i < distmap.size(); ++i)
      {
        int64_t v = distmap[i] / 2.0;
        tmpv[i] = std::min(v, maxVal);
      }
      distance_cloud_publisher_.publish(createPointCloud(*map_, tmpv));
    }

    if (distance_cloud_publisher_.getNumSubscribers() > 0)
    {
      distance_arrows_publisher_.publish(createPoseArray(*map_, paths.prevmap));
    }

    return paths;
  }

  FrontiersPriorityQueue FrontiersStrategy::calculateFrontiers(const DistanceStruct& paths) const
  {
    FrontiersPriorityQueue frontiers;

    const bool publish_frontiers = frontier_markers_publisher_.getNumSubscribers() > 0;
    const DistanceMap distmap = paths.distance;

    for (Frontier frontier : findFrontiers(*map_))
    {
      index_t goal = frontier.getGoal(*map_, distmap);
      double distance = (goal >= 0) ? distmap[goal] : UNREACHABLE;
      int size = frontier.size();
      if (isReachable(distance))
      {
        const int RminGoalDist = std::ceil(1.0 / map_->resolution);
        double score;
        if (distance < RminGoalDist)
        {
          score = -1.0;  // lowest priority possible
        }
        else
        {
          score = (size * 10) / distance;
        }
        frontiers.push(std::make_pair(score, frontier));
        ROS_INFO_STREAM(" - Evaluated frontier of size " << size
                         << " at distance " << distance << ": " << score);
      }
      else
      {
        ROS_INFO_STREAM(" - Discarded unreachable frontier of size " << size << ".");
      }

      if (publish_frontiers)
      {
        tf::Vector3 pos = map_->getPositionFromIdx(frontier.getCentroidFrontierCell());
        (*frontier_markers_)["frontier_size"].addTextMarker(
              std::to_string(size),
              pos);
        (*frontier_markers_)["frontier_distance"].addTextMarker(
              !isReachable(distance) ? "N/A" : intToHuman(int64_t(distance)),
              pos + tf::Point(0.16, 0.16, 0));
      }
    }

    if (publish_frontiers)
    {
      ROS_INFO("Publishing updated frontier information.");
      frontier_markers_->publish();
    }

    return frontiers;
  }

  std::vector<Frontier> FrontiersStrategy::findFrontiers(const GridMap& map) const
  {
    static const int8_t kFrontier = 100;
    static const int8_t kDilated = 50;
    static const int8_t kClustered = -1;

    // Find all frontier cells
    std::vector<int8_t> frontier_cells(map.size(), 0);
    int c = 0;
    for (index_t idx = 0; idx < map.size(); ++idx)
    {
      bool is_frontier = map.data[idx] < GridMap::UNREACHABLE && map.isFrontier(idx);
      frontier_cells[idx] = (is_frontier) ? kFrontier : 0;
      if (is_frontier)
      {
        ++c;
        for (index_t nidx : map.getEightNeighbours(idx))
        {
          frontier_cells[nidx] = std::max(kDilated, frontier_cells[nidx]);
        }
      }
    }
    ROS_INFO_STREAM("Found " << c << " frontier cells.");
    frontier_map_publisher_.publish(createOccupancyGrid(map, frontier_cells));

    // Group adjacent frontier cells, using a DFS.
    std::vector<Frontier> frontiers;
    for (index_t idx = 0; idx < map.size(); ++idx)
    {
      if (frontier_cells[idx] == kFrontier)
      {
        Frontier frontier;

        std::vector<index_t> stack;
        stack.push_back(idx);
        while (!stack.empty())
        {
          index_t cidx = stack.back(); stack.pop_back();

          // Don't add dilated cells to the frontier cluster.
          if (frontier_cells[cidx] == kFrontier)
          {
            FrontierCell cell(map, cidx);
            frontier.addCell(cell);
          }
          frontier_cells[cidx] = kClustered;

          // 8-neighborhood
          for (index_t nidx : map.getEightNeighbours(cidx))
          {
            if (frontier_cells[nidx] == kFrontier || frontier_cells[nidx] == kDilated)
              stack.push_back(nidx);
          }
        }

        // Frontier big enough for the robot?
        if (frontier.checkSize(neglect_area_ / map.resolution))
        {
          frontiers.push_back(frontier);
        }
      }
    }
    frontier_cloud_publisher_.publish(createPointCloud(map, frontiers));

    ROS_INFO_STREAM("Found " << frontiers.size() << " interesting frontiers.");
    return frontiers;
  }

  void FrontiersStrategy::preempt()
  {
    // Cancel any pending goals from us.
    if (own_active_goal_)
    {
      move_base_client_.cancelGoal();
      own_active_goal_ = false;
    }
  }

}  // namespace exploration
}  // namespace pal
