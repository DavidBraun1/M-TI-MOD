#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class MOD_TB3 : public rclcpp::Node
{
public:
    MOD_TB3(): Node("tb3_slalom")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MOD_TB3::odom_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MOD_TB3::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&MOD_TB3::control_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int total = msg->ranges.size();
                
        front_dist = get_min_distance(msg, 0, 15, total);      // ±15° vorne
        min_dist = get_min_distance(msg, 0, 180, total);         // 360° herum
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Vorne: %.2f und min: %.2f", front_dist, min_dist);
    }

    double get_min_distance(const sensor_msgs::msg::LaserScan::SharedPtr& msg, 
                           int angle_deg, int range_deg, int total)
    {
        // Konvertiere Winkel zu Index (0° = Index 0 ist vorne)
        int center_idx = (angle_deg * total) / 360;
        int range_width = (range_deg * total) / 360;
        
        double min_dist = msg->range_max;
        
        for (int i = center_idx - range_width; i <= center_idx + range_width; i++) {
            int idx = (i + total) % total; // Wrap around
            if (msg->ranges[idx] > msg->range_min && msg->ranges[idx] < msg->range_max) {
                min_dist = std::min(min_dist, (double)msg->ranges[idx]);
            }
        }
        
        return min_dist;
    }

    void control_loop()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();

        // Abstand zur Stange vor dem Slalom
        const double PILLER_DIST = 0.3;

        // Zustandsmaschine erst wenn erste Messung da war
        if(min_dist != 999.0){
            if(front_dist > min_dist){
                cmd.twist.linear.x = 0.0;
                cmd.twist.angular.z = 0.1;
            }
            else if (front_dist > PILLER_DIST) {
                // FALL 1: Stange zu weit weg
                RCLCPP_INFO(this->get_logger(), "Stange zu weit weg (%.2f)", front_dist);
                cmd.twist.linear.x = 0.2;
                cmd.twist.angular.z = 0.0; // Negative = rechts drehen
            }
            else {
                // FALL 2: Vor Stange stehen bleiben            
                cmd.twist.linear.x = 0.0;
                cmd.twist.angular.z = 0.0;
                
                RCLCPP_INFO(this->get_logger(), "%.2f vor der Stange zum Stehen gekommen.", 
                        PILLER_DIST);
            }
        }

        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_yaw_ = 0.0;
    double front_dist = 999.0;
    double min_dist = 999.0;
};

int main(int argc, char **argv)
{
    printf("Starte Slalom!\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MOD_TB3>());
    rclcpp::shutdown();
    return 0;
}