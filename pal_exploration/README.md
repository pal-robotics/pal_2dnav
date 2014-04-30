This node implements frequency-based, frontier-based exploration.


It takes an input an OccupancyGrid and position (eg. from OpenKarto) and sends commands to move_base, which must be configured to support planning into unknown space (note: this may not work if the costmaps are configured as "voxel" type).

If another node send a goal to move_base, pal_exploration will wait for the goal to complete, and then continue exploring from wherever the robot is; this allows semi-autonomous exploration, with an operator providing some guidance. It's also possible to pause and resume the exploration process at any time (eg. using the action interface).


HOW TO RUN EXPLORATION

 $ rosrun pal_exploration pal_exploration

 Then send an empty goal to start the action, eg. using axclient:
  $ rosrun actionlib axclient.py /exploration_server

 ... or using axcli:
  $ axcli /exploration_server "{}"


RELEVANT READING

 * Brian Yamauchi (1997). A Frontier-Based Approach for Autonomous Exploration.
 * Francesco Amigoni, Alberto Quattrini Li, Dirk Holz (2013). Evaluating the Impact of Perception and Decision Timing on Autonomous Robotic Exploration.
