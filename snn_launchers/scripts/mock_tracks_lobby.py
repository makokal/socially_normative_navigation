#!/usr/bin/env python
#
# Publishes fake tracked persons and the corresponding detections at
# /spencer/perception/tracked_persons and /spencer/perception/detected_persons.

import rospy
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from math import cos, sin, pi, radians


def create_tracked_person(track_id, x, y, theta):
    tracked_person = TrackedPerson()

    theta = radians(theta) + pi / 2.0

    tracked_person.track_id = track_id
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

    tracked_person.pose.pose.position.x = x
    tracked_person.pose.pose.position.y = y

    tracked_person.pose.pose.orientation.x = quaternion[0]
    tracked_person.pose.pose.orientation.y = quaternion[1]
    tracked_person.pose.pose.orientation.z = quaternion[2]
    tracked_person.pose.pose.orientation.w = quaternion[3]

    tracked_person.pose.covariance[0 + 0 * 6] = 0.001  # x
    tracked_person.pose.covariance[1 + 1 * 6] = 0.001  # y
    tracked_person.pose.covariance[2 + 2 * 6] = 999999  # z
    tracked_person.pose.covariance[3 + 3 * 6] = 999999  # x rotation
    tracked_person.pose.covariance[4 + 5 * 6] = 999999  # y rotation
    tracked_person.pose.covariance[4 + 5 * 6] = 999999  # z rotation

    tracked_person.twist.twist.linear.x = cos(theta)
    tracked_person.twist.twist.linear.y = sin(theta)

    for i in range(0, 3):
        tracked_person.twist.covariance[i + i * 6] = 1.0  # linear velocity
    for i in range(3, 6):
        tracked_person.twist.covariance[
            i + i * 6] = float("inf")  # rotational velocity

    return tracked_person


def main():
    # Main code
    trackPublisher = rospy.Publisher(
        '/spencer/perception/tracked_persons', TrackedPersons)

    rospy.init_node('mock_tracks_lobby')
    rate = rospy.Rate(10)

    seqCounter = 0
    while not rospy.is_shutdown():

        tracked_persons = TrackedPersons()
        tracked_persons.header.seq = seqCounter
        tracked_persons.header.frame_id = "odom"
        tracked_persons.header.stamp = rospy.Time.now()

        tracked_persons.tracks.append(create_tracked_person(01, 5, 4, 90))
        tracked_persons.tracks.append(
            create_tracked_person(02, 6, 5.45878, 90))
        tracked_persons.tracks.append(
            create_tracked_person(03, 7.22, 5.70, 90))
        tracked_persons.tracks.append(
            create_tracked_person(04, 2 + 7.22, 7.33, 90))

        tracked_persons.tracks.append(
            create_tracked_person(05, 2 + 8.92, 8.42, 90))
        tracked_persons.tracks.append(
            create_tracked_person(06, 2 + 7.92, 10.41, 90))
        tracked_persons.tracks.append(
            create_tracked_person(07, 2 + 7.2, 9.44, 90))

        tracked_persons.tracks.append(
            create_tracked_person(8, 2 + 7, 14 - 2, 90))
        tracked_persons.tracks.append(
            create_tracked_person(9, 2 + 6, 15.4123 - 2, 90))
        tracked_persons.tracks.append(
            create_tracked_person(10, 5 - 1, 18.595 - 5, 280))
        tracked_persons.tracks.append(
            create_tracked_person(11, 5 - 1, 20 - 5, 270))
        tracked_persons.tracks.append(
            create_tracked_person(12, 6 - 1, 21.5491 - 5, 240))
        tracked_persons.tracks.append(
            create_tracked_person(13, 7.48044 - 1, 19 - 5, 90))
        tracked_persons.tracks.append(
            create_tracked_person(14, 6, 24.5463, 45))
        tracked_persons.tracks.append(create_tracked_person(15, 8, 28, 90))
        tracked_persons.tracks.append(
            create_tracked_person(16, 10.4458, 23, 68))
        tracked_persons.tracks.append(
            create_tracked_person(17, 11.5004, 27, 88))
        tracked_persons.tracks.append(
            create_tracked_person(18, 14, 25.4389, 20))
        tracked_persons.tracks.append(create_tracked_person(19, 15, 21, 90))
        tracked_persons.tracks.append(
            create_tracked_person(20, 15, 22.4308, 92))
        tracked_persons.tracks.append(
            create_tracked_person(21, 15.4676, 24, 91))
        tracked_persons.tracks.append(
            create_tracked_person(22, 16.5423, 25.4178, 90))
        tracked_persons.tracks.append(create_tracked_person(23, 18, 20, 90))
        tracked_persons.tracks.append(
            create_tracked_person(24, 18.5532, 21.5011, 90))
        tracked_persons.tracks.append(
            create_tracked_person(25, 15.4739, 16.5314, 45))
        tracked_persons.tracks.append(
            create_tracked_person(26, 20, 25.5746, 90))
        tracked_persons.tracks.append(
            create_tracked_person(27, 21.5327, 24, 90))
        tracked_persons.tracks.append(
            create_tracked_person(28, 22, 26.4632, 90))
        tracked_persons.tracks.append(create_tracked_person(29, 21, 18, 45))
        tracked_persons.tracks.append(
            create_tracked_person(30, 23, 20.4335, 90))
        tracked_persons.tracks.append(
            create_tracked_person(31, 23.4972, 21.4055, 90))
        tracked_persons.tracks.append(
            create_tracked_person(32, 23.4025, 22.4749, 90))
        tracked_persons.tracks.append(
            create_tracked_person(33, 24.5281, 18.5868, 54))
        tracked_persons.tracks.append(
            create_tracked_person(34, 16.554, 3.40568 - 2, 94))
        tracked_persons.tracks.append(create_tracked_person(35, 16, 6 - 1, 94))
        tracked_persons.tracks.append(create_tracked_person(36, 20, 4, 0))
        tracked_persons.tracks.append(create_tracked_person(37, 19, 12, 25))
        tracked_persons.tracks.append(create_tracked_person(38, 23, 8, 50))
        tracked_persons.tracks.append(create_tracked_person(39, 24, 10, 90))
        tracked_persons.tracks.append(create_tracked_person(40, 25, 12, 120))
        tracked_persons.tracks.append(
            create_tracked_person(41, 7.51, 22.41, 80))
        tracked_persons.tracks.append(
            create_tracked_person(42, 8.21, 25.7, 81))
        tracked_persons.tracks.append(
            create_tracked_person(43, 3.31, 27.7, 81))
        tracked_persons.tracks.append(
            create_tracked_person(44, 11.421, 18.7, 75))
        tracked_persons.tracks.append(
            create_tracked_person(45, 25.21, 27.0, 85))
        tracked_persons.tracks.append(
            create_tracked_person(46, 18.23, 6.87, -91))
        tracked_persons.tracks.append(
            create_tracked_person(47, 18.6, 8.90, -90))
        tracked_persons.tracks.append(
            create_tracked_person(48, 20.4, 7.87, 85))
        tracked_persons.tracks.append(
            create_tracked_person(49, 15.684, 10.74, 75))
        tracked_persons.tracks.append(
            create_tracked_person(50, 15.72, 14.51, 70))

        trackPublisher.publish(tracked_persons)

        seqCounter += 1
        rate.sleep()


if __name__ == '__main__':
    main()
