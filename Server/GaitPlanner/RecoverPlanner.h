#ifndef RECOVER_PLANNER_H
#define RECOVER_PLANNER_H

#include <aris.h>
#include "Kinematics/RobotDefinitions.h"
#include "Kinematics/LegKinematics.h"

namespace robot_app
{
    // for recovery
    struct RecoverParam final : aris::server::GaitParamBase
    {
    public:
        double t_retract;
        double t_extend;
        double t_wait;
    };

    class RecoverPlanner
    {
    public:
        static const int MOTION_NUM = kinematics::MOTION_NUM;
        static void setMotionSelector(const aris::server::MotionSelector &selector);
        static int  recover(aris::model::Model &model, aris::server::PlanParamBase &param);
        static bool recoverParser(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out);
    
    private:
        static aris::server::MotionSelector motion_selector_;
        static double begin_position_[MOTION_NUM];
        static double retract_position_[MOTION_NUM];
        static double recover_position_[MOTION_NUM];
        static double current_position_[MOTION_NUM];
        static double retract_tip_position_[kinematics::LEG_DOF];
        static double recover_tip_position_[kinematics::LEG_DOF];
        static double recover_waist_position_;
    };
}

#endif
