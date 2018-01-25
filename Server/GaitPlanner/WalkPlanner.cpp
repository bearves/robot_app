#include "WalkPlanner.h"

namespace robot_app
{
    aris::server::MotionSelector WalkPlanner::motion_selector_ = nullptr;
    int WalkPlanner::total_count_;
    double WalkPlanner::begin_foot_pos_[18];
    double WalkPlanner::begin_body_pos_[3];
    double WalkPlanner::foot_pos_[18];
    double WalkPlanner::body_pos_[3];
    double WalkPlanner::begin_joint_position_[kinematics::MOTION_NUM];
    double WalkPlanner::foot_cmd_pos_[kinematics::MOTION_NUM];
    double WalkPlanner::joint_cmd_pos_[kinematics::MOTION_NUM];
    double WalkPlanner::pos2count_ratio_[kinematics::MOTION_NUM];
    double WalkPlanner::cubic_coefs1_[4];
    double WalkPlanner::cubic_coefs2_[4];

    void WalkPlanner::setMotionSelector(const aris::server::MotionSelector &selector)
    {
        motion_selector_ = selector;
    }

    int WalkPlanner::walk(aris::model::Model &model, aris::server::PlanParamBase &param)
    {
        using aris::control::EthercatMotion;
        using kinematics::PI;

        auto& wk_param = static_cast<WalkParam &>(param);
        static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

        /////////////////////////////////////////////////////////////
        // Initialize beginning states here                        //
        /////////////////////////////////////////////////////////////
        if (wk_param.count == 0)
        {
            // get pos2count ratio and begin joint position for each motor
            for (int i = 0; i < kinematics::MOTION_NUM; i++)
            {
                pos2count_ratio_[i] = cs.controller().motionAtAbs(i).pos2countRatio();
                begin_joint_position_[i] = (*wk_param.motion_raw_data)[i].feedback_pos / pos2count_ratio_[i];
            }

            // calculate cubic coefficients for body trj interpolation
            total_count_ = (int)(wk_param.period * cs.getControlFreq());
            double t[2]={0, wk_param.period};
            double x[2]={0, wk_param.step_length / 4};
            double v1[2]={0, 0.5 * wk_param.step_length / wk_param.period};
            double v2[2]={v1[1], 0};
            calculate_coe(t, x, v1, cubic_coefs1_);
            calculate_coe(t, x, v2, cubic_coefs2_);

            // set body's begin pos
            std::fill(begin_body_pos_, begin_body_pos_+3, 0);

            // set leg's begin pos
            for (int i = 0; i < kinematics::ACTIVE_LEG_NUM; i++)
            {
                kinematics::Leg::LegFK(
                    &(begin_joint_position_[i*2]), 
                    &(begin_foot_pos_[i*3]), 
                    kinematics::LEG_ORIENTATION[i]);
                begin_foot_pos_[i*3+2] = 0;

                rt_printf("Leg %d's begin position is (%.3f, %.3f)\n", 
                    i,
                    begin_foot_pos_[i*3], 
                    begin_foot_pos_[i*3+1]);
            }
            std::copy(begin_foot_pos_, begin_foot_pos_+18, foot_pos_);
            std::copy(begin_body_pos_, begin_body_pos_+3,  body_pos_);
        }

        ///////////////////////////////////////////////////////////////////
        // User Planning code begins here                                //
        // You need to calculate joint_cmd_pos_ in this part             //
        ///////////////////////////////////////////////////////////////////
        
        int period_count = wk_param.count % total_count_;
        double s = -PI / 2 * std::cos( PI * (period_count + 1) *1.0 / total_count_) + PI / 2;
        
        if (period_count == 0)
        {
            std::copy(foot_pos_, foot_pos_+18, begin_foot_pos_);
            std::copy(body_pos_, body_pos_+3,  begin_body_pos_);
        }
        
        double current_step_len;
        const int* swing_leg_group;
        const int* stance_leg_group;
        double t_r = (period_count + 1) * 1.0 / cs.getControlFreq();
        double t_r2 = t_r * t_r;
        double t_r3 = t_r2 * t_r;
        
        // trajectory planning
        if (wk_param.count / total_count_ == 0)
        {
            // the first step
            current_step_len = wk_param.step_length/2;
            swing_leg_group = kinematics::LEG_GROUP_A;
            stance_leg_group = kinematics::LEG_GROUP_B;

            // body interp(accelerate stage)
            body_pos_[0] = begin_body_pos_[0] + cubic_coefs1_[0] * t_r3 + cubic_coefs1_[1] * t_r2 +
                                   cubic_coefs1_[2] * t_r + cubic_coefs1_[3];
        }
        else if (wk_param.count/total_count_ == 2 * wk_param.step_number - 1)
        {
            //the last step
            current_step_len = wk_param.step_length/2;
            swing_leg_group = kinematics::LEG_GROUP_B;
            stance_leg_group = kinematics::LEG_GROUP_A;

            //body interp(decelerate stage)
            body_pos_[0] = begin_body_pos_[0] + cubic_coefs2_[0] * t_r3 + cubic_coefs2_[1] * t_r2 +
                                   cubic_coefs2_[2] * t_r + cubic_coefs2_[3];
        }
        else if (wk_param.count/total_count_ % 2 == 1) //constant velocity stage
        {
            current_step_len = wk_param.step_length;
            swing_leg_group = kinematics::LEG_GROUP_B;
            stance_leg_group = kinematics::LEG_GROUP_A;
            //body interp(constant velocity stage)
            body_pos_[0] = begin_body_pos_[0] + (wk_param.step_length/2)*((period_count+1)*1.0/total_count_);
        }
        else
        {
            current_step_len = wk_param.step_length;
            swing_leg_group = kinematics::LEG_GROUP_A;
            stance_leg_group = kinematics::LEG_GROUP_B;
            //body interp(constant velocity
            body_pos_[0] = begin_body_pos_[0] + (wk_param.step_length/2)*((period_count+1)*1.0/total_count_);
        }

        // plan leg pos in the global coordinate
        for (int i = 0; i < 3; i++)
        {
            int leg_id = swing_leg_group[i];
            foot_pos_[leg_id*3+0]=begin_foot_pos_[leg_id*3+0] + current_step_len*(1-std::cos(s))/2.0;
            foot_pos_[leg_id*3+1]=begin_foot_pos_[leg_id*3+1] + wk_param.step_height * std::sin(s);  
            foot_pos_[leg_id*3+2]=begin_foot_pos_[leg_id*3+2];  
        }
        for (int i = 0; i < 3; i++)
        {
            int leg_id = stance_leg_group[i];
            foot_pos_[leg_id*3+0]=begin_foot_pos_[leg_id*3+0]; 
            foot_pos_[leg_id*3+1]=begin_foot_pos_[leg_id*3+1]; 
            foot_pos_[leg_id*3+2]=begin_foot_pos_[leg_id*3+2]; 
        }

        // Leg joint cmd
        for (int i = 0; i < kinematics::ACTIVE_LEG_NUM; i++)
        {
            // Calculate foot position related to the hip coordinate
            foot_cmd_pos_[i*2] = foot_pos_[i*3] - body_pos_[0];
            foot_cmd_pos_[i*2+1] = foot_pos_[i*3+1] - body_pos_[1];

            // Calculate joint position
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[i*2]), 
                &(joint_cmd_pos_[i*2]), 
                kinematics::LEG_ORIENTATION[i]);
        }
        // Waist joint cmd
        joint_cmd_pos_[kinematics::WAIST_INDEX] = begin_joint_position_[kinematics::WAIST_INDEX];

        ///////////////////////////////////////////////////////
        // User Planning code ends here                      //
        ///////////////////////////////////////////////////////

        // don't change the following part unless you DO know what you are doing
        for (int i = 0; i < kinematics::MOTION_NUM; i++)
        {
            if (wk_param.active_motor[i])
            {
                (*wk_param.motion_raw_data)[i].cmd = EthercatMotion::Cmd::RUN;
                (*wk_param.motion_raw_data)[i].mode = EthercatMotion::Mode::POSITION;
                (*wk_param.motion_raw_data)[i].target_pos = joint_cmd_pos_[i] * pos2count_ratio_[i];
            }
        }

        // return 0 if planning finished, otherwise return positive value
        return 2 * wk_param.step_number * total_count_ - wk_param.count -1;
    }

    bool WalkPlanner::walkParser(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)
    {
        WalkParam param;
        motion_selector_(params, param);
        for (auto i : params)
        {
            if (i.first == "step_length")
            {
                param.step_length = std::stod(i.second);
            }
            else if (i.first == "step_height")
            {
                param.step_height = std::stod(i.second);
            }
            else if (i.first == "step_number")
            {
                param.step_number = std::stoi(i.second);
            }
            else if (i.first == "turning_rate")
            {
                param.turning_rate = std::stod(i.second);
            }
            else if (i.first == "period")
            {
                param.period = std::stod(i.second);
            }
        }
        msg_out.copyStruct(param);
        return true;
    }

    // supporting functions for walk gait
    void WalkPlanner::calculate_coe(double* t, double* x, double* v, double* coe)
    {
        double t1 = t[0];
        double t2 = t[1];
        double x1 = x[0];
        double x2 = x[1];
        double v1 = v[0];
        double v2 = v[1];
        double dt = t1 - t2;
        coe[0] = ((t1 - t2) * (v1 + v2) - 2 * x1 + 2 * x2)/dt/dt/dt;
        coe[1] = (-t1*t1 * (v1 + 2 * v2) + t1 * (-t2 * v1 + t2 * v2 + 3 * x1 - 3 * x2) +  t2 * (2 * t2 * v1 + t2 * v2 + 3 * x1 - 3 * x2))/dt/dt/dt;
        coe[2] = (1/dt/dt/dt) * (-t2*t2*t2 * v1 + t1*t1*t1 * v2 + t1*t1 * t2 * (2 * v1 + v2) -   t1 * t2 * (t2 * v1 + 2 * t2 * v2 + 6 * x1 - 6 * x2));
        coe[3] = (1/dt/dt/dt) * (t2 * (t1 * (-t1 + t2) * (t2 * v1 + t1 * v2) - t2 * (-3 * t1 + t2) * x1) + t1*t1 * (t1 - 3 * t2) * x2);
    }
}