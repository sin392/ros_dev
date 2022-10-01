#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TwistPublisher{
public:
  TwistPublisher() : nh_(), pnh_("~") {
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
  }

  void joyCallback(const sensor_msgs::Joy& joy_msg) {
    last_joy_ = joy_msg;
  }

  void timerCallback(const ros::TimerEvent& e) {
    int assign_x = 1;
    int assign_y = 0;
    int assign_z_left = 4;
    int assign_z_right = 5;

    float max_x = 1;
    float max_y = 1;
    float max_z = 1;

    geometry_msgs::Twist cmd_vel;
    if(0 <= assign_x && assign_x < last_joy_.axes.size()){
      cmd_vel.linear.x = max_x * last_joy_.axes[assign_x];
    }
    if(0 <= assign_y && assign_y < last_joy_.axes.size()){
      cmd_vel.linear.y = max_y * last_joy_.axes[assign_y];
    }
    if(0 <= assign_z_left  && assign_z_left  < last_joy_.buttons.size() &&
       0 <= assign_z_right && assign_z_right < last_joy_.buttons.size()){
      int kaiten = last_joy_.buttons[assign_z_left] - last_joy_.buttons[assign_z_right];
      cmd_vel.angular.z = max_z * kaiten;
    }
    cmd_pub_.publish(cmd_vel);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;
  sensor_msgs::Joy last_joy_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_twist_publisher");
  TwistPublisher twist_publisher;
  ros::spin();
  return 0;
}
