#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class MOD_TB3 : public rclcpp::Node
{
public:
    MOD_TB3(): Node("tb3_control"), turning_mode_(false), obstacle_detected_(false)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MOD_TB3::odom_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MOD_TB3::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&MOD_TB3::control_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Aktuelle Orientierung aus Quaternion zu Yaw (in Rad)
        double qx = msg->pose.pose.orientation.x; // = q1
        double qy = msg->pose.pose.orientation.y; // = q2
        double qz = msg->pose.pose.orientation.z; // = q3
        double qw = msg->pose.pose.orientation.w; // = q0 (Realteil der Quaternion)

        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = *msg;

        // Prüfen, ob vorn Hindernis im ±15°-Bereich
        int total = msg->ranges.size();
        int center = 0; // Messwinkel für Fahrt in Vorwärtsrichtung
        int range_width = total / 24; // ca. ±15°

        obstacle_detected_ = false;
        for (int i = center - range_width; i <= center + range_width; i++) {
            if (msg->ranges[i%total] < 0.5 && msg->ranges[i%total] > msg->range_min) {
                obstacle_detected_ = true;
                break;
            }
        }

        if (obstacle_detected_ && !turning_mode_) {
            double best_angle = find_best_direction(msg);
            target_yaw_ = normalize_angle(current_yaw_ - best_angle);
            turning_mode_ = true;
        }
    }

    double find_best_direction(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
    {
        // Scan in Sektoren aufteilen
        int num_sectors = 36; // je 10°
        int readings_per_sector = msg->ranges.size() / num_sectors;

        double best_avg = 0.0;
        int best_sector = 0;

        for (int s = 0; s < num_sectors; s++) {
            double sum = 0.0;
            int count = 0;
            for (int i = 0; i < readings_per_sector; i++) {
                int idx = s * readings_per_sector + i;
                if (msg->ranges[idx] > msg->range_min && msg->ranges[idx] < msg->range_max) {
                    sum += msg->ranges[idx];
                    count++;
                }
            }
            if (count > 0) {
                double avg = sum / count;
                if (avg > best_avg) {
                    best_avg = avg;
                    best_sector = s;
                }
            }
        }

        double angle_per_sector = (msg->angle_max - msg->angle_min) / num_sectors;
        double best_angle = msg->angle_min + (best_sector + 0.5) * angle_per_sector;

        RCLCPP_INFO(this->get_logger(), "Beste Richtung: %.1f° (frei: %.2f m)",
                    best_angle * 180.0 / M_PI, best_avg);

        return best_angle;
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double shortest_angular_distance(double from, double to)
    {
        double diff = normalize_angle(to - from);
        return diff;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (turning_mode_) {
            double diff = shortest_angular_distance(current_yaw_, target_yaw_);

            if (std::fabs(diff) > 0.05) { // Toleranz 0.05 rad ≈ 3°
                cmd.angular.z = (diff > 0 ? 0.5 : -0.5);
                cmd.linear.x = 0.0;
            } else {
                turning_mode_ = false;
                obstacle_detected_ = false;
            }
        } else {
            if (obstacle_detected_) {
                cmd.linear.x = 0.0; // warten bis Scan_callback Ziel setzt
            } else {
                cmd.linear.x = 0.2;
                cmd.angular.z = 0.0;
            }
        }

        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool obstacle_detected_;
    bool turning_mode_;
    double target_yaw_ = 0.0;
    double current_yaw_ = 0.0;

    sensor_msgs::msg::LaserScan last_scan_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MOD_TB3>());
    rclcpp::shutdown();
    return 0;
}