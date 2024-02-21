/**
 * @file wl_controller_base.cpp
 * @brief
 * @version 1.0
 * @date 2023-12-11
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */
#include "wl_controller_base.h"

#ifndef M_PI
#define M_PI ( 3.14159265358979323846 )
#endif

ControllerBase::ControllerBase()
    : robot_cmd_(16)
{
  loadParameters();
  wl_ = limxsdk::WheelLegged::getInstance();
  wl_->subscribeRobotState(std::bind(&ControllerBase::robotStateCallback, this, std::placeholders::_1));
}

ControllerBase::~ControllerBase()
{
}

void ControllerBase::startLoop()
{
  auto loop = [&]()
  {
    std::chrono::time_point<std::chrono::steady_clock> tp;
    tp = std::chrono::steady_clock::now();
    while (true)
    {
      update();
      tp += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_));
      std::this_thread::sleep_until(tp);
    }
  };
  th_loop_ = std::thread(loop);
}

void ControllerBase::loadParameters()
{
  joint_kp_.resize(robot_cmd_.Kp.size());
  joint_kp_ = Eigen::VectorXd::Constant(robot_cmd_.Kp.size(), 600);
  joint_kd_.resize(robot_cmd_.Kd.size());
  joint_kd_ = Eigen::VectorXd::Constant(robot_cmd_.Kd.size(), 4.5);

  damping_kd_.resize(robot_cmd_.Kd.size());
  damping_kd_ << 4.5, 4.5, 4.5, 0.6,
      4.5, 4.5, 4.5, 0.6,
      4.5, 4.5, 4.5, 0.6,
      4.5, 4.5, 4.5, 0.6;

  joint_offset_.resize(robot_cmd_.q.size());
  joint_offset_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  home_q_.resize(robot_cmd_.q.size());
  home_q_ << 0.5, 1.3, 0, 0,
      0.5, 1.3, 0, 0,
      -0.5, 1.3, 0, 0,
      -0.5, 1.3, 0, 0;

  stand_q_.resize(robot_cmd_.q.size());
  stand_q_ << 0, -0.6, 0.6, 0,
      0, -0.6, 0.6, 0,
      0, -0.6, 0.6, 0,
      0, -0.6, 0.6, 0;
}

void ControllerBase::zeroTorque()
{
  std::fill(robot_cmd_.dq.begin(), robot_cmd_.dq.end(), 0);
  std::fill(robot_cmd_.tau.begin(), robot_cmd_.tau.end(), 0);
  std::fill(robot_cmd_.Kp.begin(), robot_cmd_.Kp.end(), 0);
  std::fill(robot_cmd_.Kd.begin(), robot_cmd_.Kd.end(), 0);
  publish(robot_cmd_);
}

void ControllerBase::damping()
{
  std::fill(robot_cmd_.dq.begin(), robot_cmd_.dq.end(), 0);
  std::fill(robot_cmd_.Kp.begin(), robot_cmd_.Kp.end(), 0);
  robot_cmd_.Kd.assign(damping_kd_.data(), damping_kd_.data() + damping_kd_.size());
  publish(robot_cmd_);
}

void ControllerBase::homing(double v)
{
  Eigen::VectorXd q(robot_cmd_.q.size());
  for (uint16_t i = 0; i < q.size(); i++)
  {
    q[i] = robotState().q[i];
  }
  goPos(q, home_q_, v, dt_);
  damping();
}

void ControllerBase::standing(double v)
{
  Eigen::VectorXd q(robot_cmd_.q.size());
  for (uint16_t i = 0; i < q.size(); i++)
  {
    q[i] = robotState().q[i];
  }
  goPos(q, stand_q_, v, dt_);
}

void ControllerBase::sitDown(double v)
{
  homing(v);
}

double ControllerBase::calcCos(double start, double stop, double T, double t) const
{
  double A = (stop - start) / 2.0;
  return A * -cos(M_PI / T * t) + start + A;
}

void ControllerBase::goPos(const Eigen::VectorXd q, const Eigen::VectorXd q_d, double speed, double dt)
{
  Eigen::VectorXd duration = (q_d - q) / speed;
  double t_max = duration.maxCoeff();
  if (t_max < 0.5)
  {
    t_max = 0.5;
  }

  double t = 0;
  std::chrono::time_point<std::chrono::steady_clock> tp;
  tp = std::chrono::steady_clock::now();
  while (true)
  {
    for (uint16_t i = 0; i < q.size(); i++)
    {
      robot_cmd_.q[i] = calcCos(q[i], q_d[i], t_max, t);
      robot_cmd_.Kp[i] = joint_kp_[i];
      robot_cmd_.Kd[i] = joint_kd_[i];
    }
    publish(robot_cmd_);

    t += dt;
    if (t >= t_max)
    {
      return;
    }

    tp += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
    std::this_thread::sleep_until(tp);
  }
}

limxsdk::RobotState ControllerBase::robotState()
{
  limxsdk::RobotState state;
  mtx_.lock();
  state = robot_state_;
  mtx_.unlock();
  return state;
}

void ControllerBase::publish(limxsdk::RobotCmd &msg)
{
  for (uint16_t i = 0; i < msg.q.size(); i++)
  {
    msg.q[i] = msg.q[i] + joint_offset_[i];
  }
  wl_->publishRobotCmd(msg);
}

void ControllerBase::robotStateCallback(const limxsdk::RobotStateConstPtr &msg)
{
  mtx_.lock();
  robot_state_ = *msg;
  for (uint16_t i = 0; i < msg->q.size(); i++)
  {
    robot_state_.q[i] = msg->q[i] - joint_offset_[i];
  }
  mtx_.unlock();
  if (!recv_)
  {
    recv_ = true;
  }
}
