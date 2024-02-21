/**
 * @file pointfoot_example.cpp
 * @brief
 * @version 1.0
 * @date 2023-12-11
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include <thread>
#include <mutex>
#include <vector>
#include "limxsdk/pointfoot.h"

#ifdef WIN32
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

// Robot control frequency
static const int32_t ROBOT_CMD_RATE = 1000;

// Robot cmd state
std::vector<double> q_state_cmd_ = {0.0, 1.0, 1.0, 
                                    0.0, -1.0, -1.0};
std::vector<double> q_pos_init_(6, 0.0);



// Algorithm implementation
class AlgorithmImpl {
private:
  std::mutex mtx_;
  limxsdk::PointFoot *pf_;

  limxsdk::RobotCmd robot_cmd_;

  bool robotstate_on_;
  limxsdk::RobotState robot_state_;

  bool is_first_enter_{true};
  double time_start_{0.0};
  double time_action_ = 3.0;
  int running_iter_{1};

public:
  AlgorithmImpl() { 
    pf_ = limxsdk::PointFoot::getInstance();

    robotstate_on_ = false;

    robot_cmd_ = limxsdk::RobotCmd(pf_->getMotorNumber());

    robot_state_ = limxsdk::RobotState(pf_->getMotorNumber());

    pf_->subscribeRobotState([&](const limxsdk::RobotStateConstPtr& msg) {
      mtx_.lock();
      robot_state_ = *msg;
      robotstate_on_ = true;
      mtx_.unlock();
    });
  }

  virtual ~AlgorithmImpl() { 

  }

  void run() {
    while (true) {
      mtx_.lock();
      if (robotstate_on_ && is_first_enter_) {
        is_first_enter_ = false;
        for (int i = 0; i < robot_state_.q.size(); ++i)
        {
          q_pos_init_[i] = robot_state_.q[i];
        }
        robotstate_on_ = false;
      }
      mtx_.unlock();
   
      if ((running_iter_ > 1) && (running_iter_ <= time_action_ * ROBOT_CMD_RATE) && (!is_first_enter_))
      {
        double r = (time_action_ * ROBOT_CMD_RATE - running_iter_) / (time_action_ * ROBOT_CMD_RATE);
        std::vector<float> qPosDes(6, 0.0), dqPosDes(6, 0.0), Kp(6, 40.0), Kd(6, 3), currentDes(6, 0.0);

        for (int i = 0; i < qPosDes.size(); ++i)
        {
          qPosDes[i] = r * q_pos_init_[i] + (1 - r) * q_state_cmd_[i];
        }
        robot_cmd_.q = qPosDes;
        robot_cmd_.dq = dqPosDes;
        robot_cmd_.Kp = Kp;
        robot_cmd_.Kd = Kd;
        robot_cmd_.tau = currentDes;
      }
      running_iter_ += 1;
      
      pf_->publishRobotCmd(robot_cmd_);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};

int main(int argc, char* argv[]) {

  limxsdk::PointFoot *pf = limxsdk::PointFoot::getInstance();

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }
  if (!pf->init(robot_ip))
  {
    exit(1);
  }

  AlgorithmImpl algo;
  algo.run();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

#ifdef WIN32
  timeEndPeriod(1);
#endif
  return 0;
}
