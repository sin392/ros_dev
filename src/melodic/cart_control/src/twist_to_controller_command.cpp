#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

const double EPS = 1e-5;
const double max_speed = 5;

class CommandPublisher{
public:
  CommandPublisher() : nh_(), pnh_("~") {
    rl_pub_ = nh_.advertise<std_msgs::Float64>("/cart/right_left_wheel_controller/command", 1);
    rr_pub_ = nh_.advertise<std_msgs::Float64>("/cart/right_right_wheel_controller/command", 1);
    lr_pub_ = nh_.advertise<std_msgs::Float64>("/cart/left_right_wheel_controller/command", 1);
    ll_pub_ = nh_.advertise<std_msgs::Float64>("/cart/left_left_wheel_controller/command", 1);
    twist_sub_ = nh_.subscribe("/cmd_vel", 10, &CommandPublisher::twistCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &CommandPublisher::timerCallback, this);
  }

  void twistCallback(const geometry_msgs::Twist& msg) {
    cmd_vel_ = msg;
  }

  void timerCallback(const ros::TimerEvent& e) {
    double x = cmd_vel_.linear.x;
    double y = cmd_vel_.linear.y;
    double r = cmd_vel_.angular.z;

    geometry_msgs::Twist cmd_vel;
    std_msgs::Float64 rl, rr, lr, ll;

    if(abs(r) > EPS) {
      rl.data = max_speed * r;
      rr.data = max_speed * r;
      lr.data = - max_speed * r;
      ll.data = - max_speed * r;
    } else if(abs(x) > EPS and abs(y) > EPS) {
      if(y > 0) {
        rr.data = max_speed * x;
        lr.data = max_speed * x;
      } else {
        rl.data = max_speed * x;
        ll.data = max_speed * x;
      }
    } else {
      if(abs(x) > EPS) {
        rl.data = max_speed * x;
        rr.data = max_speed * x;
        lr.data = max_speed * x;
        ll.data = max_speed * x;
      } else {
        rl.data = - max_speed * y;
        rr.data = max_speed * y;
        lr.data = max_speed * y;
        ll.data = - max_speed * y;
      }
    }

    rl_pub_.publish(rl);
    rr_pub_.publish(rr);
    lr_pub_.publish(lr);
    ll_pub_.publish(ll);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher rl_pub_, rr_pub_, lr_pub_, ll_pub_;
  ros::Subscriber twist_sub_;
  ros::Timer timer_;
  geometry_msgs::Twist cmd_vel_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_to_controller_command");
  CommandPublisher command_publisher;
  ros::spin();
  return 0;
}
