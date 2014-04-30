pal_nav_utils
=============

The pal_nav_utils stack provides navigation-related utility methods.

 * algorithms: generic operations on grids (eg. mask operations, Dijkstra algorithm)

 * conversions: functions for converting between data types (eg. geometry_msgs::Pose <-> ::Vector3)

 * grid: class for holding a square grid of int8_t

 * grid_map: specialization of Grid for gridmaps (w/ conversion between indices and coordinates)

 * markers: utility class for easily managing a MarkerArray

 * visualization: visualization-related utility functions (eg. creating OccupancyGrid or PointCloud2 from GridMap)
