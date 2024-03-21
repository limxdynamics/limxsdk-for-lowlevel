/**
 * @file pf_joint_move.cpp
 * @brief
 * @version 1.0
 * @date 2024-3-6
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include "pf_controller_base.h"

class PFJointMove : public PFControllerBase
{
public:
    void init()
    {
        std::vector<float> offset;
        std::vector<float> limit;
        
        if (pf_->getJointOffset(offset))
        {
            joint_offset_ << offset[0], offset[1], offset[2], 
                             offset[3], offset[4], offset[5];
        }
        if (pf_->getJointLimit(limit))
        {
            joint_limit_ << limit[0], limit[1], limit[2], 
                            limit[3], limit[4], limit[5]; 
        }
    }
    void starting()
    {
        std::cout << "Waiting to receive data...\n";
        while (true)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
            double jointPos = 0;
            double r = 0.0;
            if (robotstate_on_ && is_first_enter_)
            {
                joint_init_pos_ = robot_state_.q[joint_id];
                is_first_enter_ = false;
                std::cout << "Received\n";
            }

            r = std::min(std::max(double(running_iter_) / 2000.0, 0.0), 1.0);
            jointPos = (1 - r) * joint_init_pos_ + r * joint_targetPos;
            singleJointController(joint_id, joint_kp, joint_kd, jointPos, joint_targetVel, joint_targetTorque);
            
            std::this_thread::sleep_until(time_point);
            if (!is_first_enter_)
            {
                running_iter_++;
            }
        }
    }

private:
    int joint_id = 0; 
    double joint_kp = 60;
    double joint_kd = 3; 
    double joint_targetPos = 0.7;
    double joint_targetVel = 0; 
    double joint_targetTorque = 0;  

    double joint_init_pos_ = 0;
    bool is_first_enter_{true};
    int running_iter_{1};
};



int main(int argc, char* argv[])
{
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

    PFJointMove ctrl;
    ctrl.init();
    ctrl.starting();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    #ifdef WIN32
    timeEndPeriod(1);
    #endif
    return 0;
}