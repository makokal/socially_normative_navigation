#!/usr/bin/env python
#
# Publishes fake annotations for blow out tests

import rospy

from spencer_mapping_msgs.msg import Annotation, Annotations
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import Marker, MarkerArray

from math import sin, cos


def visualize_annotations(all_annotations, pub):
    all_markers = MarkerArray()
    for annot in all_annotations.annotations:
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = rospy.Time.now()

        marker.type = Marker.LINE_STRIP
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.scale.x = 0.1

        for p in annot.shape:
            marker.points.append(p)

        all_markers.markers.append(marker)

    pub.publish(all_markers)


def visualize_annotations_orientation(all_annotations, pub):
    all_markers = MarkerArray()
    for annot in all_annotations.annotations:
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = rospy.Time.now()

        marker.type = Marker.ARROW
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.scale.x = 1
        marker.scale.x = 0.1
        marker.scale.x = 0.1

        marker.points.append(Point(annot.center.x,
                             annot.center.y,
                             0))
        marker.points.append(Point(annot.center.x + 2*cos(annot.center.theta),
                             annot.center.y + 2*sin(annot.center.theta),
                             0))

        all_markers.markers.append(marker)

    pub.publish(all_markers)


def main():
    # Main node
    annot_pub = rospy.Publisher('/spencer/mapping/map_annotations',
                                Annotations, queue_size=1)

    visual_pub = rospy.Publisher('/spencer/mapping/visual_annotations',
                                 MarkerArray, queue_size=1)

    arrow_pub = rospy.Publisher('/spencer/mapping/annotations_orienation',
                                MarkerArray, queue_size=1)

    rospy.init_node('mock_annotations')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        map_annotations = Annotations()
        map_annotations.header.frame_id = "odom"
        map_annotations.header.stamp = rospy.Time.now()

        tv1 = Annotation()
        tv1.annot_id = 0
        tv1.name = 'TV1'
        tv1.category = 'TV'
        tv1.center = Pose2D(2.1, 5.5, 0.01)
        tv1.shape.append(Point(2, 4, 0))
        tv1.shape.append(Point(2, 9, 0))
        tv1.shape.append(Point(2.2, 9, 0))
        tv1.shape.append(Point(2.2, 4, 0))

        map_annotations.annotations.append(tv1)
        annot_pub.publish(map_annotations)

        visualize_annotations(map_annotations, visual_pub)
        visualize_annotations_orientation(map_annotations, arrow_pub)

        rate.sleep()

if __name__ == '__main__':
    main()
