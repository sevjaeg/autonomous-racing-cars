#!/usr/bin/env python
import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Safety(object):
    """
    The class that handles emergency braking.
    """
    # With TRESHOLD = 0.5 the Emergency Breaking works pretty well expect in reverse
    # TRESHOLD = 0.5
    # With TRESHOLD = 1 it also works in reverse
    TRESHOLD = 1
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.acker = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
        self.bool = rospy.Publisher('brake_bool', Bool, queue_size=10)

        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)


    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

        # If the car is stopped, then publish False to deactivate the emergency breaking
        if self.speed == 0:
            # Boolean message
            boolean = Bool()
            boolean.data = False
            self.bool.publish(boolean)

    def scan_callback(self, scan_msg):
        angle = scan_msg.angle_min
        i = 0
        while angle < scan_msg.angle_max:
            beam_position = scan_msg.ranges[i]
            beam_speed = self.speed * math.cos(angle)
            if beam_speed <= 0:
                TTC = float('inf')
            else:
                TTC = beam_position / (beam_speed)
            i = i + 1
            angle = angle + scan_msg.angle_increment

            #Starting Treshold calculated as velocity/acceleration
            if TTC < self.TRESHOLD:
                rospy.loginfo("TTC under the treshold. TTC: " + str(TTC) + ", Treshold: " + str(self.TRESHOLD) + ".")
                rospy.loginfo("Activating emergincy brake")

                #AckermannDriveStamped message
                ack_drive = AckermannDrive()
                ack_stamped = AckermannDriveStamped()
                ack_drive.steering_angle = 0
                ack_drive.steering_angle_velocity = 0
                ack_drive.speed = 0
                ack_drive.acceleration = 0
                ack_drive.jerk = 0
                ack_stamped.drive = ack_drive
                self.acker.publish(ack_stamped)

                #Boolean message
                boolean = Bool()
                boolean.data = True
                self.bool.publish(boolean)

                break

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
