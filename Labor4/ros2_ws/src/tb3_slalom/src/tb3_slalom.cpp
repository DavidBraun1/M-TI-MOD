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
        current_yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int total = msg->ranges.size();
                
        front_dist = get_min_distance(msg, 0, 5, total);      // ±5° vorne
        min_dist = get_min_distance(msg, 0, 180, total);         // 360° herum
        front_min_dist = get_min_distance(msg, 0, 90, total);     // 180° vorne
        
        //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        //                     "Vorne: %.2f und min: %.2f", front_dist, min_dist);
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
                min_idx = idx;
            }
        }
        
        return min_dist;
    }

    double angle_diff(double a, double b) {
        return std::atan2(std::sin(a - b), std::cos(a - b));
    }

    void control_loop()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();

        if (test < 50){
            cmd.twist.linear.x = 0.2;
            cmd.twist.angular.z = 0.0;
            test += 1;
        }
        else{
            cmd.twist.linear.x = 0.2;
            cmd.twist.angular.z = 1.1;
        }

/*
        // Abstand zur Stange vor dem Slalom
        const double PILLER_DIST = 0.3;

        //Halbkreis
        const double radius = 0.4;
        const double v = 0.15; //Geschwindigkeit
        const double w = v / radius; //Winkelgesch

        switch(state){
            case 0: //Zur Säule fahren
            {
                //wait for bootup
                if (min_dist == 999.9){
                
                }
                else{
                    if(front_dist > min_dist){
                        cmd.twist.linear.x = 0.0;
                        cmd.twist.angular.z = -dir * 0.2;
                        RCLCPP_INFO(this->get_logger(), "Front Distanz: %.2f, Min Distanz: %.2f", front_dist, min_dist);
                    }
                    else if(front_dist > PILLER_DIST){
                        cmd.twist.linear.x = 0.2;
                        cmd.twist.angular.z = 0.0;
                    }
                    else{
                        cmd.twist.linear.x = 0.0;
                        cmd.twist.angular.z = 0.0;
                        state = 2;
                        RCLCPP_INFO(this->get_logger(), "State 0 abgeschlossen");
                    }
                }
                break;
                }
            case 1: //nach einer Säule zur nächsten fahren
            {
                if(front_dist > front_min_dist){
                    cmd.twist.linear.x = 0.0;
                    cmd.twist.angular.z = -dir * 0.2;
                    RCLCPP_INFO(this->get_logger(), "Front Distanz: %.2f, Front Min Distanz: %.2f", front_dist, front_min_dist);              
                }
                else if(front_dist > PILLER_DIST){
                    cmd.twist.linear.x = 0.2;
                    cmd.twist.angular.z = 0.0;                    
                }
                else{
                    cmd.twist.linear.x = 0.0;
                    cmd.twist.angular.z = 0.0;
                    state = 2;
                    RCLCPP_INFO(this->get_logger(), "State 1 abgeschlossen");
                    slalom_i += 1;
                }
                break;
            }
            case 2: //Um 90° drehen
            {
                if (!init) {
                start_yaw = current_yaw;
                init = true;
                RCLCPP_INFO(this->get_logger(), "Init gemacht");
                }

                double delta_yaw = current_yaw - start_yaw;

                if(std::abs(delta_yaw) < M_PI_2){
                    cmd.twist.linear.x = 0.0;
                    cmd.twist.angular.z = dir * 0.2;
                }
                else{
                    cmd.twist.linear.x = 0.0;
                    cmd.twist.angular.z = 0.0;
                    init = false;
                    state = 3;
                    RCLCPP_INFO(this->get_logger(), "State 2 abgeschlossen");
                    dir = dir * -1;
                }
                break;
            }
            case 3: //Halbkreis fahren
            {
                if (!init) {
                start_yaw = current_yaw;
                init = true;
                RCLCPP_INFO(this->get_logger(), "Init gemacht");
                }
                
                double dyaw = angle_diff(current_yaw, start_yaw);
                
                if (slalom_i == 3 && std::abs(dyaw) >= M_PI/2){
                    cmd.twist.linear.x  = 0.0;
                    cmd.twist.angular.z = 0.0;
                    state = 5;
                    RCLCPP_INFO(this->get_logger(), "State 3 abgeschlossen");
                    break;
                }

                if(std::abs(dyaw) < M_PI-0.05){
                    cmd.twist.linear.x  = v;
                    cmd.twist.angular.z = dir * w;
                }
                else{
                    cmd.twist.linear.x  = 0.0;
                    cmd.twist.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "State 3 abgeschlossen");
                    state = 4;
                    init = false;
                }
                break;
            }
            case 4: //Anhalten und 45° zurück drehen
            {
                if (!init) {
                    start_yaw = current_yaw;
                    init = true;
                    RCLCPP_INFO(this->get_logger(), "Init gemacht");
                }

                double delta_yaw = current_yaw - start_yaw;

                if(std::abs(delta_yaw) < M_PI_2/2){
                    cmd.twist.linear.x  = 0.0;
                    cmd.twist.angular.z = -dir * 0.2;
                }else{
                    cmd.twist.linear.x  = 0.0;
                    cmd.twist.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "State 4 abgeschlossen");
                    state = 1;
                    init = false;
                }
                break;
            }
            case 5:
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Ende");
            }
        }
    */
        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_yaw = 0.0;
    double front_dist = 999.0;
    double min_dist = 999.0;
    double front_min_dist = 999.0;
    int min_idx = -1;
    //Halbkreis
    int state = 0;
    double radius;
    bool init = false;
    double start_yaw = 0.0;
    double dir = -1; // Negative = rechts drehen
    int slalom_i = 0;
    int test = 0;
    };

int main(int argc, char **argv)
{
    printf("Starte Slalom!\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MOD_TB3>());
    rclcpp::shutdown();
    return 0;
}