from os.path import expanduser

WHOLE_ROOT_PATH = "{}/catkin_ws".format(expanduser('~'))
WHOLE_OUTPUTS_PATH = "{}/outputs".format(WHOLE_ROOT_PATH)
PKG_ROOT_PATH = "{}/src/myrobot_moveit".format(WHOLE_ROOT_PATH)
PKG_OUTPUTS_PATH = "{}/outputs".format(PKG_ROOT_PATH)