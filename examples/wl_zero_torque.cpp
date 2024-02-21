/**
 * @file wheel legged zero torque
 *
 * @brief This file contains the wheel legged zero torque robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include "wl_controller_base.h"

class WheelLeggedZeroTorque : public ControllerBase
{
public:
  void starting()
  {
    zeroTorque();
  }

private:
  void update()
  {
  }
};

int main(int argc, char *argv[])
{
  limxsdk::WheelLegged *wl = limxsdk::WheelLegged::getInstance();

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }
  if (!wl->init(robot_ip))
  {
    exit(1);
  }

  WheelLeggedZeroTorque ctl;
  ctl.starting();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
