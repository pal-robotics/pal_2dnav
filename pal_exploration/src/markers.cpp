/**************************************************************************
**
**  markers.cpp
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 11-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#include <utility>

#include <pal_exploration/markers.h>

namespace pal
{

namespace exploration
{

  void Markers::setDefaultColor(float r, float g, float b, float a)
  {
    color_r_ = r;
    color_g_ = g;
    color_b_ = b;
    color_a_ = a;
  }

  visualization_msgs::Marker& Markers::addMarker(const tf::Point& position)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns_;
    marker.id = marker_array_.markers.size();
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = 0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = color_r_;
    marker.color.g = color_g_;
    marker.color.b = color_b_;
    marker.color.a = 1.0;

    marker_array_.markers.push_back(marker);
    return marker_array_.markers.back();
  }

  visualization_msgs::Marker& Markers::addTextMarker(const std::string& text, const tf::Point& position)
  {
    visualization_msgs::Marker& marker = addMarker(position);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = text;
    return marker;
  }

  void Markers::publish(void)
  {
    int num_markers = marker_array_.markers.size();

    visualization_msgs::Marker deleteMarker;
    deleteMarker.header.frame_id = "map";
    deleteMarker.header.stamp = ros::Time::now();
    deleteMarker.ns = ns_;
    deleteMarker.action = visualization_msgs::Marker::DELETE;

    for (int i = marker_array_.markers.size(); i < last_num_markers_; ++i)
    {
      deleteMarker.id = i;
      deleteMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker_array_.markers.push_back(deleteMarker);
    }

    publisher_.publish(marker_array_);
    marker_array_.markers.clear();
    last_num_markers_ = num_markers;
  }

  void MarkersCollection::addNamespace(const std::string& ns)
  {
    markers_.insert(make_pair(ns, Markers(publisher_, ns)));
  }

  void MarkersCollection::addNamespace(const std::string& ns, float r, float g, float b)
  {
    Markers marker(publisher_, ns);
    marker.setDefaultColor(r, g, b, 1.0);
    markers_.insert(make_pair(ns, marker));
  }

  Markers& MarkersCollection::operator[](const std::string& ns)
  {
    return markers_.at(ns);
  }

  void MarkersCollection::publish(void)
  {
    for (std::pair<std::string, Markers> entry : markers_)
    {
      markers_.at(entry.first).publish();
    }
  }

}  // namespace exploration
}  // namespace pal
