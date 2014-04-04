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

#ifndef PAL_EXPLORATION_RANGE_ITERATOR_
#define PAL_EXPLORATION_RANGE_ITERATOR_

#include <iterator>

#include <pal_nav_utils/grid_map.h>

/** ********************************** **/
/**          CURRENTLY UNUSED!         **/
/** ********************************** **/

namespace pal
{

namespace exploration
{

  template<typename T>
  struct NumericRangeIterator
    : std::iterator<std::bidirectional_iterator_tag, T>
  {
  public:
    NumericRangeIterator(T start, T end, T incr)
      : start_value_(start), end_value_(end), current_value_(start), incr_(incr)
    {
    }

    T operator*() {
      return current_value_;
    }

    inline NumericRangeIterator& operator++() {
      current_value_ += incr_;
      return *this;
    }

    inline NumericRangeIterator& operator--() {
      current_value_ -= incr_;
      return *this;
    }

    inline NumericRangeIterator begin() {
      return NumericRangeIterator(start_value, end_value, start_value, incr);
    }

    inline NumericRangeIterator end() {
      return NumericRangeIterator(start_value, end_value, end_value, incr);
    }

    inline friend bool operator==(const GridMapIterator& lhs, const GridMapIterator& rhs)
    {
      return lhs.current_value == rhs.current_value;
    }

  private:
    NumericRangeIterator(T start, T end, T current, T incr)
      : start_value_(start), end_value_(end), current_value_(current), incr_(incr)
    {
    }

    T start_value_;
    T end_value_;
    T current_value_;
    T incr_;
  };

  inline NumericRangeIterator<index_t> iterateGridMapRow(const GridMap& map, uint32_t row)
  {
    index_t start = row * map.width;
    return NumericRangeIterator<index_t>(start, start + map.width, 1);
  }

  inline NumericRangeIterator<index_t> iterateGridMapColumn(const GridMap& map, uint32_t col)
  {
    return NumericRangeIterator<index_t>(0, map.size() - (map.width - col), map.width);
  }

}  // namespace exploration
}  // namespace pal

#endif  // PAL_EXPLORATION_RANGE_ITERATOR_
