#pragma once

#include <ros/ros.h>

// Message (std_msgs)
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std_msgs;

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits_interface.h>

#include "denso_robot_core/denso_robot_core.h"
#include "denso_robot_core/denso_controller.h"
#include "denso_robot_core/denso_robot.h"
#include "denso_robot_core/denso_variable.h"
#include "denso_robot_core/UserIO.h"
using namespace denso_robot_core;

#include <boost/thread.hpp>

#define JOINT_MAX (8)

namespace cart_control
{
class CartHW : public hardware_interface::RobotHW
{
public:
  CartHW();
  virtual ~CartHW();

  HRESULT Initialize();

  ros::Time getTime() const
  {
    return ros::Time::now();
  }

  ros::Duration getPeriod() const
  {
    return m_ctrl->get_Duration();
  }

  void read(ros::Time, ros::Duration);
  void write(ros::Time, ros::Duration);

  bool isSlaveSyncMode() const;

private:
  HRESULT ChangeModeWithClearError(int mode);
  void Callback_ChangeMode(const Int32::ConstPtr& msg);

  HRESULT CheckRobotType();

  bool hasError();
  void printErrorDescription(HRESULT error_code, const std::string& error_message);

private:
  hardware_interface::JointStateInterface m_JntStInterface;
  hardware_interface::PositionJointInterface m_PosJntInterface;
  double m_cmd[JOINT_MAX]; 
  double m_pos[JOINT_MAX]; // 現在のジョイント値(処理後)
  double m_vel[JOINT_MAX];
  double m_eff[JOINT_MAX];
  int m_type[JOINT_MAX]; // ジョイントタイプ, 0->スライド, 1->回転
  std::vector<double> m_joint; // 現在のジョイント値(処理前)

  DensoRobotCore_Ptr m_eng;
  DensoController_Ptr m_ctrl;
  DensoRobot_Ptr m_rob;
  DensoVariable_Ptr m_varErr;

  std::string m_robName;
  int m_robJoints;
  int m_sendfmt;
  int m_recvfmt;

  ros::Subscriber m_subChangeMode;

  ros::Publisher m_pubCurMode;

  boost::mutex m_mtxMode;
};

}  // namespace denso_robot_control