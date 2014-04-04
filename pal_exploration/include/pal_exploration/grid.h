/**************************************************************************
**
**  grid.h
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 07-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef PAL_EXPLORATION_GRID_
#define PAL_EXPLORATION_GRID_

#include <cstdint>
#include <vector>

#include <ros/assert.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

namespace pal
{

namespace exploration
{

  typedef int32_t index_t;
  typedef std::vector<index_t> indices_t;

  class Grid
  {
  public:
    Grid(const std::vector<int8_t>& data, int width, int height)
      : data(data), width(width), height(height)
    {}

    virtual ~Grid() {}

    inline int size(void) const
    {
      return width * height;
    }

    /*
     * Converts a cell position expressed as (x, y) into an index.
     *
     * If the position is invalid, returns -1.
     */
    inline index_t getIdx(int x, int y) const
    {
      index_t idx = x + y * width;
      if (idx < 0 || idx >= size())
        return -1;
      return idx;
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

    indices_t getFourNeighbours(index_t idx) const;

    indices_t getEightNeighbours(index_t idx) const;

    std::vector<int8_t> data;
    int width;
    int height;
  };

  // A mask/kernel to use for convolution, etc.
  typedef Grid GridMask;

}  // namespace exploration
}  // namespace pal

#endif  // PAL_EXPLORATION_GRID_
