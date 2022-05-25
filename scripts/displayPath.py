#!/usr/bin/python3
import csv
import rospy
from collections import defaultdict
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from time import sleep

colorCount = 0

class DataLoader:
    def __init__(self, filename):
        self.filename = filename
        self.data = defaultdict(list)
        self.read()
    def read(self):
        with open(self.filename, mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            line_count = 0
            max_y = max_x = 0
            for row in csv_reader:
                for key, value in row.items():
                    key = key.strip()
                    val = float(value)
                    if key == 'y':
                        val *= 0.7
                        max_y = max(max_y, val)
                    else:
                        max_x = max(max_x, val)
                    self.data[key].append(val)
                line_count += 1
            print(f'Processed = {line_count} lines max_x = {max_x:.3f} lines max_y = {max_y:.3f}.')

    def __getitem__(self, item):
        return self.data.get(item)


class PathViewer(DataLoader):

    def __init__(self, filename, topic):
        super().__init__(filename)
        self._pub = rospy.Publisher(topic, Marker, queue_size=10)
    def getMarker(self):
        marker_ = Marker()
        marker_.header.frame_id = "map"
        marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.POINTS
        marker_.action = marker_.ADD
        marker_.id = 1

        for x, y in zip(self['x'], self['y']):
            point = Point()
            point.x = x
            point.y = y
            point.z = 0
            marker_.points.append(point)
        marker_.scale.x = 0.02
        marker_.scale.y = 0.02
        marker_.scale.z = 0.02
        global colorCount
        marker_.color = ColorRGBA(0, 1, colorCount, 1)
        colorCount += 1
        return marker_

    def publish(self):
        marker = self.getMarker()
        self._pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('path', anonymous=True, disable_signals=True)
    path1 = '/home/redwan/catkin_ws/src/bovo/results/goodExp/robo0.csv'
    path2 = '/home/redwan/catkin_ws/src/bovo/results/goodExp/robo1.csv'

    r1 = PathViewer(path1, '/roomba20/path')
    r2 = PathViewer(path2, 'roomba21/path')

    # print(r1['y'])
    sleep(2)
    print('publishing paths')
    r1.publish()
    r2.publish()
