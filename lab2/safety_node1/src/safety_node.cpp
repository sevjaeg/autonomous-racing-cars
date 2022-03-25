#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// Include ROS msg type headers and libraries
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <algorithm>
#include <math.h>

// Task 2.2.a
#include <std_msgs/Float64.h>

// Task 2.2.b
#include <dynamic_reconfigure/server.h>
#include <safety_node1/ThresholdsConfig.h>

double threshold_fw = 0.75;
double threshold_bw = 1.8;

// Task 2.2.b
void reconfig_callback(safety_node1::ThresholdsConfig &config, uint32_t level) {
    ROS_INFO("Reconfiguring TTC thesholds: fw: %f, bw: %f", config.threshold_fw, config.threshold_bw);
    threshold_fw = config.threshold_fw;
    threshold_bw = config.threshold_bw;
}

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    double threshold;

    // Create ROS subscribers and publishers
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_brake_;
    ros::Publisher pub_brake_bool_;

    // Task 2.2.a
    ros::Publisher pub_velocity_;
    ros::Publisher pub_ttc_;
    ros::Publisher pub_min_distance_;

public:
    Safety() {
        ROS_INFO("Hello from safety_node!");

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

        // Task 2.2.a
        pub_velocity_ = n.advertise<std_msgs::Float64>("velocity", 10);
        pub_ttc_ = n.advertise<std_msgs::Float64>("ttc", 10);
        pub_min_distance_ = n.advertise<std_msgs::Float64>("min_distance", 10);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // Update current speed and TTC threshold
        speed = odom_msg->twist.twist.linear.x;
        if(speed < 0.0) {
            threshold = threshold_bw;
        } else {
            threshold = threshold_fw;
        }

        // Task 2.2.a
        std_msgs::Float64 velocity_msg;
        velocity_msg.data = speed;
        pub_velocity_.publish(velocity_msg);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        int length = scan_msg->ranges.size();
        double* ttc = new double[length];

        // Task 2.2.a
        std_msgs::Float64 min_distance_msg;
        min_distance_msg.data = *std::min_element(&(scan_msg->ranges[0]), &(scan_msg->ranges[length-1]));
        pub_min_distance_.publish(min_distance_msg);

        // Calculate TTC
        for (int i = 0; i < length; i++) {
            if (scan_msg->ranges[i] < scan_msg->range_min || scan_msg->ranges[i] > scan_msg->range_max) {
                ttc[i] = std::numeric_limits<double>::infinity();
                continue;
            }

            double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
            double r_derivative = -speed * cos(angle); 
            ttc[i] = scan_msg->ranges[i] / std::max(-r_derivative, 0.0);

            // ROS_DEBUG("range %f at angle %f at speed %f yields ttc %f", scan_msg->ranges[i], angle, r_derivative, ttc[i]);
        }
        
        double min = *std::min_element(ttc, ttc + length);
        ROS_DEBUG("min ttc=%f", min);

        // Task 2.2.a
        std_msgs::Float64 ttc_msg;
        ttc_msg.data = min;
        pub_ttc_.publish(ttc_msg);
        
        // Publish drive/brake message
        std_msgs::Bool brake_bool_msg;
        if (min < threshold && min > 0) {
            ROS_DEBUG("threshold warning with ttc=%f", min);

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

    // Task 2.2.b
    dynamic_reconfigure::Server<safety_node1::ThresholdsConfig> server;
    dynamic_reconfigure::Server<safety_node1::ThresholdsConfig>::CallbackType f;
    f = boost::bind(&reconfig_callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}
