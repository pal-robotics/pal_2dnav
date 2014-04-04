/**************************************************************************
**
**  markers.h
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 11-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef PAL_EXPLORATION_MARKERS_
#define PAL_EXPLORATION_MARKERS_

#include <string>
#include <map>

#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

namespace pal
{

namespace exploration
{

  class Markers
  {
  public:
    Markers(ros::Publisher& pub, const std::string& ns)
      : publisher_(pub), ns_(ns), last_num_markers_(0),
        color_r_(0), color_g_(0), color_b_(0), color_a_(1.0) {}

    void setDefaultColor(float r, float g, float b, float a);

    visualization_msgs::Marker& addMarker(const tf::Point &position);

    visualization_msgs::Marker& addTextMarker(const std::string& text, const tf::Point& position);

    void publish(void);

  private:
    ros::Publisher& publisher_;
    const std::string ns_;
    visualization_msgs::MarkerArray marker_array_;
    int last_num_markers_;

    float color_r_;
    float color_g_;
    float color_b_;
    float color_a_;
  };

  class MarkersCollection
  {
  public:
    MarkersCollection(ros::Publisher& pub) : publisher_(pub) {}

    void addNamespace(const std::string& ns);
    void addNamespace(const std::string& ns, float r, float g, float b);

    Markers& operator[](const std::string& ns);

    void publish(void);

  private:
    ros::Publisher& publisher_;
    std::map<std::string, Markers> markers_;
  };

}  // namespace exploration
}  // namespace pal

#endif  // PAL_EXPLORATION_MARKERS_
