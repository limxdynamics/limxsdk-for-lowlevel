/**
 * @file wl_controller_base.h
 * @brief Controller base calss file
 * @version 1.0
 * @date 2023-12-11
 * 
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 */
#pragma once

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>
#include "limxsdk/wheel_legged.h"
#include "common/util.h"

/**
 * @brief Controller base calss
 * 
 */
class ControllerBase
{
public:
  ControllerBase();
  ~ControllerBase();

protected:
  virtual void starting() = 0;
  virtual void update() = 0;
  /**
   * @brief Start loop
   * 
   */
  void startLoop();
  /**
   * @brief Load parameters
   * 
   */
  void loadParameters();
  /**
   * @brief Publish zero torque command
   * 
   */
  void zeroTorque();
  /**
   * @brief Publish damping command
   * 
   */
  void damping();
  /**
   * @brief Homing
   * 
   * @param v The average speed of motion
   */
  void homing(double v = 1.0);
  /**
   * @brief standing
   * 
   * @param v The average speed of motion
   */
  void standing(double v = 0.3);
  /**
   * @brief Sit down
   * 
   * @param v The average speed of motion
   */
  void sitDown(double v = 0.5);

  void publish(limxsdk::RobotCmd &msg);
  limxsdk::RobotState robotState();
  void goPos(const Eigen::VectorXd q, const Eigen::VectorXd q_d, double speed, double dt);

  limxsdk::WheelLegged *wl_;
  limxsdk::RobotCmd robot_cmd_;
  bool recv_{false};
  double dt_{2e-3};

private:
  double calcCos(double start, double stop, double T, double t) const;
  void robotStateCallback(const limxsdk::RobotStateConstPtr &msg);

  limxsdk::RobotState robot_state_;
  std::thread th_loop_;
  std::mutex mtx_;

  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;
  Eigen::VectorXd damping_kd_;
  Eigen::VectorXd joint_offset_;
  Eigen::VectorXd home_q_;
  Eigen::VectorXd stand_q_;
};
