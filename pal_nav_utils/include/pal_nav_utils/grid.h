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

#ifndef PAL_NAV_UTILS_GRID_
#define PAL_NAV_UTILS_GRID_

#include <cstdint>
#include <vector>

#include <ros/assert.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

namespace pal
{

namespace nav
{

  typedef int32_t index_t;
  typedef std::vector<index_t> indices_t;

  template <typename T>
  class Grid
  {
  public:
    typedef T value_type;

    Grid(int width, int height)
      : data(width*height), width(width), height(height)
    {}

    Grid(const std::vector<value_type>& data, int width, int height)
      : data(data), width(width), height(height)
    {}

    virtual ~Grid() {}

    inline int size(void) const
    {
      return width * height;
    }

    /*
     * Same as data[getIdx(x, y)].
     */
    inline value_type cell(int x, int y) const
    {
      return data[getIdx(x, y)];
    }

    /*
     * Converts a cell position expressed as (x, y) into an index.
     *
     * No boundary checks are performed.
     */
    inline index_t getIdx(int x, int y) const
    {
      return x + y * width;
    }

    /*
     * Returns a vector indicating the direction to go from idx_a to idx_b.
     */
    inline tf::Vector3 getGradient(index_t idx_a, index_t idx_b) const
    {
      tf::Vector3 vec(idx_b % width - idx_a % width,
                      idx_b / width - idx_a / width,
                      0);
      return vec;
    }

    indices_t getFourNeighbours(index_t idx) const
    {
      indices_t neighbours;

      int x = idx % width;
      int y = idx / width;

      if (x + 1 < width) neighbours.push_back(idx + 1);
      if (x > 0) neighbours.push_back(idx - 1);

      if (y + 1 < height) neighbours.push_back(idx + width);
      if (y > 0) neighbours.push_back(idx - width);

      return neighbours;
    }

    indices_t getEightNeighbours(index_t idx) const
    {
      indices_t neighbours = getFourNeighbours(idx);

      int x = idx % width;
      int y = idx / width;

      if (y + 1 < height)
      {
        if (x + 1 < width) neighbours.push_back(idx + width + 1);
        if (x > 0) neighbours.push_back(idx + width - 1);
      }

      if (y > 0)
      {
        if (x + 1 < width) neighbours.push_back(idx - width + 1);
        if (x > 0) neighbours.push_back(idx - width - 1);
      }

      return neighbours;
    }

    std::vector<value_type> data;
    int width;
    int height;
  };

  // A mask/kernel to use for convolution, etc.
  template <typename T>
  struct GridMask
  {
    typedef Grid<T> type;
  };

}  // namespace nav
}  // namespace pal

#endif  // PAL_NAV_UTILS_GRID_
