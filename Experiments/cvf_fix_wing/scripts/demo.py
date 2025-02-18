#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('setpoint_position_node', anonymous=True)
    setpoint_pub = rospy.Publisher('/typhoon_h480_0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    rate = rospy.Rate(20)  # 20 Hz

    target_position = PositionTarget()
    target_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    target_position.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                                PositionTarget.IGNORE_YAW_RATE
    target_position.position.x = 100
    target_position.position.y = 100
    target_position.position.z = 25

    while not rospy.is_shutdown():
        setpoint_pub.publish(target_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass