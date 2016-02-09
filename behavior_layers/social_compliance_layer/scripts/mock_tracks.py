#!/usr/bin/env python
#
# Publishes fake tracked persons and the corresponding detections
# (if not occluded)

import rospy
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from math import cos, sin, radians


def createTrackedPerson(track_id, x, y, theta):
    trackedPerson = TrackedPerson()

    # theta = radians(theta) + pi / 2.0
    theta = radians(theta)

    trackedPerson.track_id = track_id
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

    trackedPerson.pose.pose.position.x = x
    trackedPerson.pose.pose.position.y = y

    trackedPerson.pose.pose.orientation.x = quaternion[0]
    trackedPerson.pose.pose.orientation.y = quaternion[1]
    trackedPerson.pose.pose.orientation.z = quaternion[2]
    trackedPerson.pose.pose.orientation.w = quaternion[3]

    trackedPerson.pose.covariance[0 + 0 * 6] = 0.001  # x
    trackedPerson.pose.covariance[1 + 1 * 6] = 0.001  # y
    trackedPerson.pose.covariance[2 + 2 * 6] = 999999  # z
    trackedPerson.pose.covariance[3 + 3 * 6] = 999999  # x rotation
    trackedPerson.pose.covariance[4 + 5 * 6] = 999999  # y rotation
    trackedPerson.pose.covariance[4 + 5 * 6] = 999999  # z rotation

    trackedPerson.twist.twist.linear.x = cos(theta)
    trackedPerson.twist.twist.linear.y = sin(theta)

    for i in range(0, 3):
        trackedPerson.twist.covariance[i + i * 6] = 1.0  # linear velocity
    for i in range(3, 6):
        trackedPerson.twist.covariance[
            i + i * 6] = float("inf")  # rotational velocity

    return trackedPerson


def main():
    # Main code
    trackPublisher = rospy.Publisher('/spencer/perception/tracked_persons',
                                     TrackedPersons, queue_size=1)

    rospy.init_node('mock_tracks')
    rate = rospy.Rate(10)

    seqCounter = 0
    while not rospy.is_shutdown():

        trackedPersons = TrackedPersons()
        trackedPersons.header.seq = seqCounter
        trackedPersons.header.frame_id = "odom"
        trackedPersons.header.stamp = rospy.Time.now()

        # trackedPersons.tracks.append(
        # createTrackedPerson( trackId, x, y, theta ) )
        trackedPersons.tracks.append(createTrackedPerson(1, 6.1, 4, 181))
        trackedPersons.tracks.append(createTrackedPerson(2, 6, 5.1, 168.7))
        trackedPersons.tracks.append(createTrackedPerson(3, 7, 7, 182))
        trackedPersons.tracks.append(createTrackedPerson(4, 6.8, 8.0, -168.7))
        trackedPersons.tracks.append(createTrackedPerson(5, 6.2, 8.7, -168.3))

        # some two loners
        trackedPersons.tracks.append(createTrackedPerson(6, 12.2, 11.7, 68.3))
        trackedPersons.tracks.append(createTrackedPerson(7, 12.1, 8.7, -45))

        trackPublisher.publish(trackedPersons)

        seqCounter += 1
        rate.sleep()


if __name__ == '__main__':
    main()
