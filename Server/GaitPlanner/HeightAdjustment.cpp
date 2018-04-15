#include "HeightAdjustment.h"

namespace robot_app
{
    aris::server::MotionSelector HeightAdjustment::motion_selector_ = nullptr;
    int HeightAdjustment::total_count_;
    double HeightAdjustment::begin_foot_pos_[18];
    double HeightAdjustment::begin_body_pos_[3];
    double HeightAdjustment::foot_pos_[18];
    double HeightAdjustment::body_pos_[3];
    double HeightAdjustment::begin_joint_position_[kinematics::MOTION_NUM];
    double HeightAdjustment::foot_cmd_pos_[kinematics::MOTION_NUM];
    double HeightAdjustment::joint_cmd_pos_[kinematics::MOTION_NUM];
    double HeightAdjustment::pos2count_ratio_[kinematics::MOTION_NUM];

    double HeightAdjustment::actual_height;
    double HeightAdjustment::tip_[18]=  {   0.445  ,  0.0,        0,//F
               0.2525,   0.0, -0.25902,//FL
             -0.2525,   0.0, -0.25902,//RL
              -0.445  ,  0.0,        0,//R
             -0.2525,   0.0,  0.25902,//RR
              0.2525,   0.0,  0.25902 };//FR
    double HeightAdjustment::body[6];

    void HeightAdjustment::setMotionSelector(const aris::server::MotionSelector &selector)
    {
        motion_selector_ = selector;
    }

    int HeightAdjustment::height(aris::model::Model &model, aris::server::PlanParamBase &param)
    {
        using aris::control::EthercatMotion;
        using kinematics::PI;

        auto& height_param = static_cast<HeightParam &>(param);
        static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

        /////////////////////////////////////////////////////////////
        // Initialize beginning states here                        //
        /////////////////////////////////////////////////////////////
        if (height_param.count == 0)
        {
            // get pos2count ratio and begin joint position for each motor
            for (int i = 0; i < kinematics::MOTION_NUM; i++)
            {
                pos2count_ratio_[i] = cs.controller().motionAtAbs(i).pos2countRatio();
                begin_joint_position_[i] = (*height_param.motion_raw_data)[i].feedback_pos / pos2count_ratio_[i];
            }

            // calculate cubic coefficients for body trj interpolation
            total_count_ = (int)(height_param.period * cs.getControlFreq());

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
            actual_height=-begin_foot_pos_[1];//the legnth of lf leg in y axis is actual height
        }

        ///////////////////////////////////////////////////////////////////
        // User Planning code begins here                                //
        // You need to calculate joint_cmd_pos_ in this part             //
        ///////////////////////////////////////////////////////////////////
        if (height_param.count>=0 && height_param.count<total_count_)
        {
            double y0=actual_height;
            double y1=height_param.height_adjustment;
            double y=0.5*(y0+y1)-0.5*(y1-y0)*std::cos( PI * (height_param.count + 1) *1.0 / total_count_);
            body[0]=0;
            body[1]=y;
            body[2]=0;
            body[3]=0;
            body[4]=0;
            body[5]=0;
            kinematics::O13Inverse::InverseSolve(
                &(tip_[0]),
                &(body[0]),
                &(joint_cmd_pos_[0]));
        }

        //joint_cmd_pos_[kinematics::WAIST_INDEX] = begin_joint_position_[kinematics::WAIST_INDEX];

        ///////////////////////////////////////////////////////
        // User Planning code ends here                      //
        ///////////////////////////////////////////////////////

        // don't change the following part unless you DO know what you are doing
        for (int i = 0; i < kinematics::MOTION_NUM; i++)
        {
            if (height_param.active_motor[i])
            {
                (*height_param.motion_raw_data)[i].cmd = EthercatMotion::Cmd::RUN;
                (*height_param.motion_raw_data)[i].mode = EthercatMotion::Mode::POSITION;
                (*height_param.motion_raw_data)[i].target_pos = joint_cmd_pos_[i] * pos2count_ratio_[i];
            }
        }

        // return 0 if planning finished, otherwise return positive value
        return  total_count_ - height_param.count -1;
    }

    bool HeightAdjustment::heightParser(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)
    {
        HeightParam param;

        if (motion_selector_)
            motion_selector_(params, param);

        for (auto i : params)
        {
            if (i.first == "height_adjustment")
            {
                param.height_adjustment = std::stod(i.second);
            }
            else if (i.first == "period")
            {
                param.period = std::stod(i.second);
            }
        }
        msg_out.copyStruct(param);
        return true;
    }
}

