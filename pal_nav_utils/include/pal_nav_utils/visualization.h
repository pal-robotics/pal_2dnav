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

#ifndef PAL_NAV_UTILS_VISUALIZATION_
#define PAL_NAV_UTILS_VISUALIZATION_

#include <vector>
#include <string>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include "pal_nav_utils/grid_map.h"

namespace pal
{

namespace nav
{

  nav_msgs::OccupancyGrid createOccupancyGrid(const GridMap& map);

  nav_msgs::OccupancyGrid createOccupancyGrid(const GridMap& map, const std::vector<int8_t>& cells);

  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<float>& cells, float height_scale=3.0);
  sensor_msgs::PointCloud2 createPointCloud(const GridMap& map, const std::vector<int8_t>& cells, float height_scale=3.0);

  /*
   * Creates a PoseArray with arrows in each cell showing which paths
   * leave from there.
   *
   * prevmap: After "index_t p = prevmap[idx];", p contains the index of
   *          the previous node for the path loading to idx.
   */
  geometry_msgs::PoseArray createPoseArray(const GridMap& map, const std::vector<index_t>& prevmap);

  inline std::string intToHuman(int64_t number)
  {
    const std::pair<int64_t, std::string> magnitudes[] = {
      std::make_pair(1e6, "M"),
      std::make_pair(1e3, "k")
    };
    for (auto m : magnitudes)
    {
      if (number > m.first)
        return std::to_string(number / m.first) + m.second;
    }
    return std::to_string(number);
  }

}  // namespace nav
}  // namespace pal

#endif  // PAL_NAV_UTILS_VISUALIZATION_
