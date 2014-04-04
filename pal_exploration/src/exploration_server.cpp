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

#include <cstring>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include "pal_exploration/ExplorationAction.h"
#include "pal_exploration/PalExplorationConfig.h"
#include "pal_exploration/frontiers_strategy.h"

using namespace pal::nav;

namespace pal
{

namespace exploration
{

  class ExplorationServer
  {
    typedef actionlib::SimpleActionServer<pal_exploration::ExplorationAction> ExplorationActionServer;

  public:
    ExplorationServer(std::string name)
        : action_server_(node_, name, boost::bind(&ExplorationServer::executeCb, this, _1), false),
          action_name_(name)
    {
      action_server_.start();
    }

    ~ExplorationServer(void) {}

    void executeCb(const pal_exploration::ExplorationGoalConstPtr& goal)
    {
      FrontiersStrategy exploration;
      dynreconf_srv_.setCallback(boost::bind(&BaseStrategy::configCb, &exploration, _1, _2));

      bool success = exploration.execute([&]() {
          if (action_server_.isPreemptRequested())
          {
              exploration.setPreempted();
          }
          return true;
      });

      pal_exploration::ExplorationResult result;
      if (success)
      {
        ROS_INFO("Exploration completed successfully.");
        action_server_.setSucceeded(result);
      }
      else
      {
        ROS_INFO("Exploration aborted.");
        action_server_.setAborted(result);
      }

      dynreconf_srv_.setCallback(NULL);
    }

  private:
    ros::NodeHandle node_;
    ExplorationActionServer action_server_;
    dynamic_reconfigure::Server<pal_exploration::PalExplorationConfig> dynreconf_srv_;
    std::string action_name_;
  };

}  // namespace exploration
}  // namespace pal


int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration_server");

  pal::exploration::ExplorationServer exploration(ros::this_node::getName());
  ros::spin();

  return 0;
}
