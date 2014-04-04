/**************************************************************************
**
**  grid.cpp
**
**  Author: Siegfried-A. Gevatter Pujals
**  Email : siegfried.gevatter@pal-robotics.com
**  Created on: 07-10-2013
**
**  Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#include <queue>
#include <utility>

#include <pal_exploration/grid.h>

namespace pal
{

namespace exploration
{

  indices_t Grid::getFourNeighbours(index_t idx) const
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

  indices_t Grid::getEightNeighbours(index_t idx) const
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

}  // namespace exploration
}  // namespace pal
