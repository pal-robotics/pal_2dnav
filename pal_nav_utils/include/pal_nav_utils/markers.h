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

#ifndef PAL_NAV_UTILS_MARKERS_
#define PAL_NAV_UTILS_MARKERS_

#include <string>
#include <map>

#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

namespace pal
{

namespace nav
{

  class Markers
  {
  public:
    Markers(ros::Publisher& pub, const std::string& ns)
      : publisher_(pub), ns_(ns), last_num_markers_(0),
        color_r_(0), color_g_(0), color_b_(0), color_a_(1.0),
        scale_x_(0.2), scale_y_(0.2), scale_z_(0.2) {}

    void setDefaultScale(float x, float y, float z);
    void setDefaultColor(float r, float g, float b, float a);

    visualization_msgs::Marker& addMarker(const tf::Point& position);
    visualization_msgs::Marker& addMarker(
        const tf::Point& position,
        const tf::Quaternion& orientation);

    visualization_msgs::Marker& addTextMarker(const std::string& text,
        const tf::Point& position);

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

    float scale_x_;
    float scale_y_;
    float scale_z_;
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

}  // namespace nav
}  // namespace pal

#endif  // PAL_NAV_UTILS_MARKERS_
