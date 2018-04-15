#include "RecoverPlanner.h"
#include <cmath>

namespace robot_app
{
    aris::server::MotionSelector RecoverPlanner::motion_selector_ = nullptr;
    double RecoverPlanner::begin_position_[MOTION_NUM];
    double RecoverPlanner::retract_position_[MOTION_NUM];
    double RecoverPlanner::recover_position_[MOTION_NUM];
    double RecoverPlanner::current_position_[MOTION_NUM];

    double RecoverPlanner::retract_tip_position_[kinematics::LEG_DOF] = {0, -kinematics::RETRACT_HEIGHT};
    double RecoverPlanner::recover_tip_position_[kinematics::LEG_DOF] = {0, -kinematics::STANDING_HEIGHT};
    double RecoverPlanner::recover_waist_position_ = 0;

    void RecoverPlanner::setMotionSelector(const aris::server::MotionSelector &selector)
    {
        motion_selector_ = selector;
    }

    int RecoverPlanner::recover(aris::model::Model &model, aris::server::PlanParamBase &param)
    {
        using aris::control::EthercatMotion;

        auto& rc_param = static_cast<RecoverParam &>(param);
        static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

        // initialize target positions for each axis
        if (rc_param.count == 0)
        {
            for (int i = 0; i < MOTION_NUM; i++)
            {
                double pos2count_ratio = cs.controller().motionAtAbs(i).pos2countRatio();
                begin_position_[i] = (*rc_param.motion_raw_data)[i].feedback_pos / pos2count_ratio;
            }
            // assign positions for leg axes
            for (int i = 0; i < kinematics::LEG_NUM; i++)
            {
                kinematics::Leg::LegIK(retract_tip_position_, &retract_position_[i*2], kinematics::LEG_ORIENTATION[i]);
                kinematics::Leg::LegIK(recover_tip_position_, &recover_position_[i*2], kinematics::LEG_ORIENTATION[i]);
            }
            rt_printf("Retract joint pos: %.3f, %.3f\n", retract_position_[0], retract_position_[1]);
            rt_printf("Recover joint pos: %.3f, %.3f\n", recover_position_[0], recover_position_[1]);
            // assign positions for waist axis
            retract_position_[kinematics::WAIST_INDEX] = begin_position_[kinematics::WAIST_INDEX];
            recover_position_[kinematics::WAIST_INDEX] = recover_waist_position_;
        }

        // stage_retract starts from the begin position, ends at the retract position
        int stage_retract_count = rc_param.t_retract * cs.getControlFreq();
        // stage_extend starts from the retract position, ends at the recover position
        int stage_extend_count = rc_param.t_extend * cs.getControlFreq();
        // stage_wait holds at the retract position 
        int stage_wait_count = rc_param.t_wait * cs.getControlFreq();
        int total_count = stage_retract_count + stage_extend_count + stage_wait_count;

        double time_ratio;
        double pivot;

        for (int i = 0; i < MOTION_NUM; i++)
        {
            if (rc_param.active_motor[i])
            {
                double pos2count_ratio = cs.controller().motionAtAbs(i).pos2countRatio();

                if (rc_param.count < stage_retract_count)
                {
                    time_ratio = rc_param.count * 1.0 / stage_retract_count;
                    pivot = (1 - std::cos(kinematics::PI * time_ratio)) / 2.0;
                    current_position_[i] = (1 - pivot) * begin_position_[i] + pivot * retract_position_[i];
                }
                else if (rc_param.count < stage_retract_count + stage_wait_count)
                {
                    current_position_[i] = retract_position_[i];
                }
                else if (rc_param.count < total_count)
                {
                    time_ratio = (rc_param.count - stage_retract_count - stage_wait_count) * 1.0 / stage_extend_count;
                    pivot = (1 - std::cos(kinematics::PI * time_ratio)) / 2.0;
                    current_position_[i] = (1 - pivot) * retract_position_[i] + pivot * recover_position_[i];
                }
                else
                {
                    current_position_[i] = recover_position_[i];
                }
                (*rc_param.motion_raw_data)[i].cmd = EthercatMotion::Cmd::RUN;
                (*rc_param.motion_raw_data)[i].mode = EthercatMotion::Mode::POSITION;
                (*rc_param.motion_raw_data)[i].target_pos = current_position_[i] * pos2count_ratio;
            }
        }

        return total_count - rc_param.count;
    }

    bool RecoverPlanner::recoverParser(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)
    {
        RecoverParam param;

        if (motion_selector_)
            motion_selector_(params, param);

        for (auto i : params)
        {
            if (i.first == "t_retract")
            {
                param.t_retract = std::stod(i.second);
            }
            else if (i.first == "t_extend")
            {
                param.t_extend = std::stod(i.second);
            }
            else if (i.first == "t_wait")
            {
                param.t_wait = std::stod(i.second);
            }
        }
        msg_out.copyStruct(param);
        return true;
    }
}

// XML configuration
/*
            <rc default="rc_param">
              <rc_param type="group" >
                <motor_select type="unique" default="all">
                  <all abbreviation="a"/>
                  <first abbreviation="f"/>
                  <second abbreviation="s"/>
                  <motor abbreviation="m" type="intArray" default="0"/>
                  <physical_motor abbreviation="p" type="intArray" default="0"/>
                  <leg abbreviation="l" type="intArray" default="0"/>
                </motor_select>
                <t_retract abbreviation="t" type="int" default="3"/>
                <t_extend abbreviation="r" type="int" default="3"/>
                <t_wait abbreviation="x" type="int" default="1"/>
              </rc_param>
            </rc>
*/
