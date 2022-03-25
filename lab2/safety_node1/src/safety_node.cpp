#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>

// Include ROS msg type headers and libraries
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <algorithm>
#include <math.h>

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    double threshold_fw;
    double threshold_bw;
    double threshold;

    // Create ROS subscribers and publishers
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_brake_;
    ros::Publisher pub_brake_bool_;

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brkake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // Create ROS subscribers and publishers
        sub_scan_ = n.subscribe("scan", 10, &Safety::scan_callback, this);
        sub_odom_ = n.subscribe("odom", 10, &Safety::odom_callback, this);

        pub_brake_ = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 10);
        pub_brake_bool_ = n.advertise<std_msgs::Bool>("brake_bool", 10);

        if(n.getParam("/safety_node/threshold_fw", threshold_fw) && n.getParam("/safety_node/threshold_bw", threshold_bw)) {
            ROS_INFO("Got TTC thresholds fw:%f, bw:%f", threshold_fw, threshold_bw);
        } else {
            ROS_ERROR("Failed to get TTC thresholds params");
        }
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // Update current speed and TTC threshold
        speed = odom_msg->twist.twist.linear.x;
        if(speed < 0.0) {
            threshold = threshold_bw;
        } else {
            threshold = threshold_fw;
        }
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        int length = scan_msg->ranges.size();
        double* ttc = new double[length];

        // Calculate TTC
        for (int i = 0; i < length; i++) {
            if (scan_msg->ranges[i] < scan_msg->range_min || scan_msg->ranges[i] > scan_msg->range_max) {
                ttc[i] = std::numeric_limits<double>::infinity();
                continue;
            }

            double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
            double r_derivative = -speed * cos(angle); 
            ttc[i] = scan_msg->ranges[i] / std::max(-r_derivative, 0.0);

            ROS_DEBUG("range %f at angle %f at speed %f yields ttc %f", scan_msg->ranges[i], angle, r_derivative, ttc[i]);
        }
        
        double* min = std::min_element(ttc, ttc + length);
        ROS_DEBUG("min ttc=%f", *min);
        
        // Publish drive/brake message
        std_msgs::Bool brake_bool_msg;
        if (*min < threshold && *min > 0) {
            ROS_INFO("threshold warning with ttc=%f", *min);

            ackermann_msgs::AckermannDriveStamped brake_msg;
            brake_msg.drive.speed = 0.0;
            pub_brake_.publish(brake_msg);
            
            brake_bool_msg.data = true;
        } else {
            brake_bool_msg.data = false;
        }
        pub_brake_bool_.publish(brake_bool_msg);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;

    ROS_INFO("Hello from safety_node!");

    ros::Rate loop_rate(40);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
