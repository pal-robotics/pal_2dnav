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

#include "pal_nav_utils/markers.h"

#include <utility>

namespace pal
{

namespace nav
{

  void Markers::setDefaultScale(float x, float y, float z)
  {
    scale_x_ = x;
    scale_y_ = y;
    scale_z_ = z;
  }

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

    marker.scale.x = scale_x_;
    marker.scale.y = scale_y_;
    marker.scale.z = scale_z_;

    marker.color.r = color_r_;
    marker.color.g = color_g_;
    marker.color.b = color_b_;
    marker.color.a = 1.0;

    marker_array_.markers.push_back(marker);
    return marker_array_.markers.back();
  }

  visualization_msgs::Marker& Markers::addMarker(const tf::Point& position, const tf::Quaternion& orientation)
  {
    visualization_msgs::Marker& marker = addMarker(position);
    tf::quaternionTFToMsg(orientation, marker.pose.orientation);
    return marker;
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

}  // namespace nav
}  // namespace pal
