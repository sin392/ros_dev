#include <string.h>
#include <sstream>
#include "cart_control/cart_hw.h"

#define RAD2DEG(x) ((x)*180.0 / M_PI)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)

#define M2MM(x) ((x)*1000.0)
#define MM2M(x) ((x) / 1000.0)

namespace cart_control
{
CartHW::CartHW()
{
  memset(m_cmd, 0, sizeof(m_cmd));
  memset(m_pos, 0, sizeof(m_pos));
  memset(m_vel, 0, sizeof(m_vel));
  memset(m_eff, 0, sizeof(m_eff));
  memset(m_type, 0, sizeof(m_type));
  m_joint.resize(JOINT_MAX);

  m_eng = boost::make_shared<DensoRobotCore>();
  m_ctrl.reset();
  m_rob.reset();
  m_varErr.reset();

  m_robName = "";
  m_robJoints = 0;
  m_sendfmt = DensoRobot::SENDFMT_NONE;
  m_recvfmt = DensoRobot::RECVFMT_POSE_J;
}

CartHW::~CartHW()
{
}

HRESULT CartHW::Initialize()
{
  ros::NodeHandle nh;

  if (!nh.getParam("robot_name", m_robName))
  {
    ROS_WARN("Failed to get robot_name parameter.");
  }

  if (!nh.getParam("robot_joints", m_robJoints))
  {
    ROS_WARN("Failed to get robot_joints parameter.");
  }

  for (int i = 0; i < m_robJoints; i++)
  {
    std::stringstream ss;
    ss << "joint_" << i + 1;
    std::string default_joint_name = ss.str();

    if (!nh.getParam(default_joint_name, m_type[i]))
    {
      ROS_WARN("Failed to get joint_%d parameter.", i + 1);
      ROS_WARN("It was assumed revolute type.");
      m_type[i] = 1;
    }

    std::string joint_name = "";
    std::string name_param_key = default_joint_name + "_name";
    nh.param(name_param_key, joint_name, default_joint_name);

    hardware_interface::JointStateHandle state_handle(joint_name, &m_pos[i], &m_vel[i], &m_eff[i]);
    m_JntStInterface.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(m_JntStInterface.getHandle(joint_name), &m_cmd[i]);
    m_PosJntInterface.registerHandle(pos_handle);
  }

  int armGroup = 0;
  if (!nh.getParam("arm_group", armGroup))
  {
    ROS_INFO("Use arm group 0.");
    armGroup = 0;
  }

  HRESULT hr = m_eng->Initialize();
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to connect real controller. (%X)", hr);
    return hr;
  }

  m_ctrl = m_eng->get_Controller();

  DensoRobot_Ptr pRob;
  hr = m_ctrl->get_Robot(DensoBase::SRV_ACT, &pRob);
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to connect real robot. (%X)", hr);
    return hr;
  }

  m_rob = pRob;

  hr = CheckRobotType();
  if (FAILED(hr))
  {
    ROS_ERROR("Invalid robot type.");
    return hr;
  }

  m_rob->ChangeArmGroup(armGroup);

  hr = m_ctrl->AddVariable("@ERROR_CODE");
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to add @ERROR_CODE object");
    return hr;
  }
  hr = m_ctrl->get_Variable("@ERROR_CODE", &m_varErr);
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to get @ERROR_CODE object");
    return hr;
  }

  hr = m_ctrl->ExecClearError();
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to clear error");
    return hr;
  }

  hr = m_ctrl->ExecResetStoState();
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to reset STO");
    return hr;
  }

  hr = m_rob->ExecCurJnt(m_joint); // m_jointに初期ジョイント値を代入
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to get current joint");
    return hr;
  }

  for (int i = 0; i < m_robJoints; i++)
  {
    switch (m_type[i])
    {
      case 0:  // prismatic
        m_pos[i] = MM2M(m_joint[i]);
        break;
      case 1:  // revolute
        m_pos[i] = DEG2RAD(m_joint[i]);
        break;
      case -1:  // fixed
      default:
        m_pos[i] = 0.0;
        break;
    }
    // m_cmd[i] = m_pos[i];
  }
  registerInterface(&m_JntStInterface);
  registerInterface(&m_PosJntInterface);

  hr = m_rob->AddVariable("@SERVO_ON");
  if (SUCCEEDED(hr))
  {
    DensoVariable_Ptr pVar;
    hr = m_rob->get_Variable("@SERVO_ON", &pVar);
    if (SUCCEEDED(hr))
    {
      VARIANT_Ptr vntVal(new VARIANT());
      vntVal->vt = VT_BOOL;
      vntVal->boolVal = VARIANT_TRUE;
      hr = pVar->ExecPutValue(vntVal);
    }
  }
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to motor on");
    return hr;
  }

  int recvfmt_type = m_recvfmt;
  switch (m_recvfmt & DensoRobot::RECVFMT_POSE)
  {
    case DensoRobot::RECVFMT_POSE_J:
    case DensoRobot::RECVFMT_POSE_PJ:
    case DensoRobot::RECVFMT_POSE_TJ:
      break;
    case DensoRobot::RECVFMT_POSE_P:
      recvfmt_type = DensoRobot::RECVFMT_POSE_PJ;
      break;
    case DensoRobot::RECVFMT_POSE_T:
      recvfmt_type = DensoRobot::RECVFMT_POSE_TJ;
      break;
    default:
      recvfmt_type = DensoRobot::RECVFMT_POSE_J;
  }
  if (recvfmt_type != m_recvfmt)
  {
    m_recvfmt = ((m_recvfmt & ~DensoRobot::RECVFMT_POSE) | recvfmt_type);
    ROS_WARN("Chagned recv_format to %d to contain joint.", m_recvfmt);
  }
  m_rob->set_SendFormat(m_sendfmt);
  m_rob->set_RecvFormat(m_recvfmt);

  m_subChangeMode = nh.subscribe<Int32>("ChangeMode", 1, &CartHW::Callback_ChangeMode, this);
  m_pubCurMode = nh.advertise<Int32>("CurMode", 1);

  hr = ChangeModeWithClearError(DensoRobot::SLVMODE_SYNC_WAIT | DensoRobot::SLVMODE_POSE_J);
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to change to slave mode");
    return hr;
  }

  return S_OK;
}

HRESULT CartHW::ChangeModeWithClearError(int mode)
{
  HRESULT hr = m_eng->ChangeMode(mode, mode == DensoRobot::SLVMODE_NONE);
  if (FAILED(hr))
  {
    // Clear Error
    HRESULT hres = m_ctrl->ExecClearError();
    if (FAILED(hres))
    {
      printErrorDescription(hres, "Failed to clear error");
    }
  }

  Int32 msg;
  msg.data = m_eng->get_Mode();
  m_pubCurMode.publish(msg);
  
  return hr;
}

void CartHW::Callback_ChangeMode(const Int32::ConstPtr& msg)
{
  boost::mutex::scoped_lock lockMode(m_mtxMode);

  ROS_INFO("Change to mode %d.", msg->data);
  HRESULT hr = ChangeModeWithClearError(msg->data);
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to change mode");
  }
}

HRESULT CartHW::CheckRobotType()
{
  DensoVariable_Ptr pVar;
  VARIANT_Ptr vntVal(new VARIANT());
  std::string strTypeName = "@TYPE_NAME";

  HRESULT hr = m_rob->AddVariable(strTypeName);
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to add @TYPE_NAME");

    return hr;
  }
  m_rob->get_Variable(strTypeName, &pVar);
  hr = pVar->ExecGetValue(vntVal);
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to get @TYPE_NAME");

    return hr;
  }

  strTypeName = DensoBase::ConvertBSTRToString(vntVal->bstrVal);

  // m_robNameとstrTypeNameが一致していなかったらエラー
  // m_robNameにはvs060.launch.xmlで記述したロボットタイプ名(robot_name)が入る
  // strTypeNameにはロボットから直接取得した？ロボットタイプ名が入る
  if (strncmp(m_robName.c_str(), strTypeName.c_str(),
              (m_robName.length() < strTypeName.length()) ? m_robName.length() : strTypeName.length()))
  {
    return E_FAIL;
  }

  return 0;
}

void CartHW::read(ros::Time time, ros::Duration period)
{
  boost::mutex::scoped_lock lockMode(m_mtxMode);

  if (m_eng->get_Mode() == DensoRobot::SLVMODE_NONE)
  {
    HRESULT hr = m_rob->ExecCurJnt(m_joint);
    if (FAILED(hr))
    {
      ROS_ERROR("Failed to get current joint. (%X)", hr);
    }
  }

  for (int i = 0; i < m_robJoints; i++)
  {
    switch (m_type[i])
    {
      case 0:  // prismatic
        m_pos[i] = MM2M(m_joint[i]);
        break;
      case 1:  // revolute
        m_pos[i] = DEG2RAD(m_joint[i]);
        break;
      case -1:  // fixed
      default:
        m_pos[i] = 0.0;
        break;
    }
  }
}

void CartHW::write(ros::Time time, ros::Duration period)
{
  boost::mutex::scoped_lock lockMode(m_mtxMode);

  if (m_eng->get_Mode() != DensoRobot::SLVMODE_NONE)
  {
    std::vector<double> pose(m_joint);
    // pose.resize(JOINT_MAX);
    int bits = 0x0000;
    for (int i = 0; i < m_robJoints; i++)
    {
      // TODO: m_cmd[i]の初期値によって最初のロボットの動作が不安定になる問題。
      // コンストラクタではm_cmd[i]は0になるようにしてあるが、
      // controllerが立ち上がるとなぜか別の値に初期化される。
      // 現時点ではコントローラーが立ち上がったのを確認してから
      // rostopic pub -1 /cart/left_left_wheel_controller/command std_msgs/Float64 "data: 0"
      // みたいな感じにコントローラに指令値0をパブリッシュするノードを作るくらいしか思いつかない

      pose[i] += m_cmd[i];
      // switch (m_type[i])
      // {
      //   case 0:  // prismatic
      //     pose[i] = M2MM(m_cmd[i]);
      //     break;
      //   case 1:  // revolute
      //     pose[i] = RAD2DEG(m_cmd[i]);
      //     break;
      //   case -1:  // fixed
      //   default:
      //     pose[i] = 0.0;
      //     break;
      // }
      bits |= (1 << i);
    }
    pose.push_back(0x400000 | bits);

    // std::cout << "#####" << std::endl;
    // for(auto i : m_cmd) std::cout << i << " ";
    // std::cout << std::endl;
    // for(auto i : pose) std::cout << i << " ";
    // std::cout << std::endl;
    // for(auto i : m_joint) std::cout << i << " ";
    // std::cout << std::endl;
    // std::cout << "#####" << std::endl;

    HRESULT hr = m_rob->ExecSlaveMove(pose, m_joint);

    // hrは符の値だとエラー
    if (FAILED(hr) && (hr != DensoRobot::E_BUF_FULL))
    {
      int error_count = 0;

      printErrorDescription(hr, "Failed to write");
      if (!hasError())
      {
        return;
      }
      ROS_ERROR("Automatically change to normal mode.");
      ChangeModeWithClearError(DensoRobot::SLVMODE_NONE);

      hr = m_ctrl->ExecGetCurErrorCount(error_count);
      if (SUCCEEDED(hr))
      {
        for (int i = error_count - 1; 0 <= i; i--)
        {
          HRESULT error_code;
          std::string error_message;

          hr = m_ctrl->ExecGetCurErrorInfo(i, error_code, error_message);
          if (FAILED(hr))
          {
            return;
          }
          ROS_ERROR("  [%d] %s (%X)", i + 1, error_message.c_str(), error_code);
        }
      }
    }
  }
}


bool CartHW::isSlaveSyncMode() const
{
  if (m_eng->get_Mode() & DensoRobot::SLVMODE_SYNC_WAIT)
  {
    return true;
  }
  return false;
}

bool CartHW::hasError()
{
  HRESULT hr;
  VARIANT_Ptr vntVal(new VARIANT());

  hr = m_varErr->ExecGetValue(vntVal);
  if (SUCCEEDED(hr) && (vntVal->lVal == 0))
  {
    return false;
  }
  return true;
}

void CartHW::printErrorDescription(HRESULT error_code, const std::string& error_message)
{
  HRESULT hr;
  std::string error_description;

  if (m_eng->get_Mode() == DensoRobot::SLVMODE_NONE)
  {
    hr = m_ctrl->ExecGetErrorDescription(error_code, error_description);
    if (SUCCEEDED(hr))
    {
      ROS_ERROR("%s: %s (%X)", error_message.c_str(), error_description.c_str(), error_code);
      return;
    }
  }
  ROS_ERROR("%s (%X)", error_message.c_str(), error_code);
}
}  // namespace cart_control