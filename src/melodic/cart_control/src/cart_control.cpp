#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "cart_control/cart_hw.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cart_control");
  ros::NodeHandle nh;

  cart_control::CartHW cart;
  HRESULT hr = cart.Initialize();
  if (SUCCEEDED(hr))
  {
    controller_manager::ControllerManager cm(&cart, nh);

    ros::Rate rate(1.0 / cart.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
      ros::Time now = cart.getTime();
      ros::Duration dt = cart.getPeriod();

      cart.read(now, dt);

      cm.update(now, dt);

      cart.write(now, dt);

      if (!cart.isSlaveSyncMode())
      {
        rate.sleep();
      }
      else
      {
        ros::spinOnce();
      }
    }
    spinner.stop();
  }

  return 0;
}