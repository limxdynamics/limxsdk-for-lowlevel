/**
 * @file wheel_legged_standup
 *
 * @brief This file contains the wheel_legged_standup of robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include "wl_controller_base.h"

class WLStandUp : public ControllerBase
{
public:
  void starting()
  {
    std::cout << "Waiting to receive data...\n";
    while (!recv_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Received\n";

    homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    standing();
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

  WLStandUp ctl;
  ctl.starting();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
