#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

last_lin_acc = Vector3()
last_ang_vel = Vector3()

def imu_msg_callback(msg, cb_args):
    pub = cb_args[0]
    global last_lin_acc
    global last_ang_vel
    send = Imu()
    send.header = msg.header
    send.header.stamp = rospy.Time.now()
    # send.header.frame_id = "camera_imu_frame"
    # if msg.linear_acceleration_covariance[0] != -1.0:
    #     last_lin_acc.x = msg.linear_acceleration.z
    #     last_lin_acc.y = msg.linear_acceleration.x * -1
    #     last_lin_acc.z = msg.linear_acceleration.y * -1
    # elif msg.angular_velocity_covariance[0] != -1.0:
    #     last_ang_vel.x = msg.angular_velocity.z
    #     last_ang_vel.y = msg.angular_velocity.x * -1
    #     last_ang_vel.z = msg.angular_velocity.y * -1

    if msg.linear_acceleration_covariance[0] != -1.0:
        last_lin_acc = msg.linear_acceleration
    elif msg.angular_velocity_covariance[0] != -1.0:
        last_ang_vel = msg.angular_velocity

    send.angular_velocity = last_ang_vel
    send.linear_acceleration = last_lin_acc

    send.orientation_covariance[0] = -1.0
    pub.publish(send)

def imu_relay():
    rospy.init_node("imu_relay")
    pub = rospy.Publisher("/imu/data", Imu, queue_size=5)
    rospy.Subscriber("/camera/imu/data_raw", Imu, callback=imu_msg_callback, callback_args=[pub]) # TODO: what is the difference between this and data_raw?

    rospy.spin()


if __name__ == '__main__':
    try:
        imu_relay()
    except rospy.ROSInterruptException as e:
        print e
