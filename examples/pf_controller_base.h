/**
 * @file pf_controller_base.h
 * @brief
 * @version 1.0
 * @date 2024-3-5
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include <thread>
#include <mutex>
#include <vector>
#include "limxsdk/pointfoot.h"
#include <Eigen/Dense>
#include <iostream>

#ifdef WIN32
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

/**
 * @brief Controller base class
 * 
 */

class PFControllerBase
{
public:
    PFControllerBase();
    ~PFControllerBase();

protected:
    /**
     * @brief move single joint
     * 
     * @param jointId The id of the joint
     * The motor order for the commnd data is as follows:
     *        0: abad_L_joint,  1: hip_L_joint,  2: knee_L_joint
     *        3: abad_R_joint,  4: hip_R_joint,  5: knee_R_joint
     */
    void singleJointController(int jointId, double kp, double kd, 
                                            double targetPos, double targetVel, 
                                            double targetTorque);
    /**
     * @brief move all of the joints
     */
    void groupJointController(std::vector<float> &kp, std::vector<float> &kd,
                              std::vector<float> &targetPos, std::vector<float> &targetVel,
                              std::vector<float> &targetTorque);
    
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
     * @brief Get the number of joints
     */
    int getNumofJoint() { return joint_num_; }
    
    const int32_t ROBOT_CMD_RATE = 1000;

    std::mutex mtx_;
    
    limxsdk::PointFoot *pf_;
    limxsdk::RobotCmd robot_cmd_;
    limxsdk::RobotState robot_state_;
    
    bool robotstate_on_;
    bool is_first_enter_{true};
    double time_start_{0.0};
    double time_action_ = 3.0;
    int running_iter_{1};

    Eigen::VectorXd joint_offset_;
    Eigen::VectorXd joint_limit_;

private:
    int joint_num_{6};

};