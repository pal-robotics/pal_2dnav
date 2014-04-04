# -*- coding: utf-8 -*-
#
# binary_map.py
#
# A script to re-publish an OccupancyGrid changing all unexplored cells
# to free cells. There should be no need to use it, it's just included
# to help debug problems with move_base.
#
# Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
#
# Authors:
#   * Siegfried-A. Gevatter Pujals

import rospy

from nav_msgs.msg import OccupancyGrid

class Main:

    def __init__(self):
        self._subscriber = rospy.Subscriber('/map', OccupancyGrid, self.on_map_received, queue_size=1)
        self._publisher = rospy.Publisher('/binary_map', OccupancyGrid, latch=True)

    def run(self):
        rospy.spin()

    def on_map_received(self, map_):
        map_.data = map(lambda x: max(0, x), map_.data)
        print 'Published /binary_map'
        self._publisher.publish(map_)

if __name__ == '__main__':
    rospy.init_node('binary_map')
    Main().run()
