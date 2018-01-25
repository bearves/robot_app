#ifndef WALK_PLANNER_H
#define WALK_PLANNER_H

#include <aris.h>
#include "Kinematics/RobotDefinitions.h"
#include "Kinematics/LegKinematics.h"

namespace robot_app
{
    // for recovery
    struct WalkParam final : aris::server::GaitParamBase
    {
    public:
        double step_length;
        double step_height;
        int step_number;
        double turning_rate;
        double period;
    };

    class WalkPlanner
    {
    public:
        static void setMotionSelector(const aris::server::MotionSelector &selector);
        static int  walk(aris::model::Model &model, aris::server::PlanParamBase &param);
        static bool walkParser(const std::string &cmd, 
                               const std::map<std::string, std::string> &params, 
                               aris::core::Msg &msg_out);
    
    private:
        static aris::server::MotionSelector motion_selector_;

        static int total_count_;
        static double begin_foot_pos_[18];
        static double begin_body_pos_[3];
        static double foot_pos_[18];
        static double body_pos_[3];
        static double foot_cmd_pos_[kinematics::MOTION_NUM];
        static double joint_cmd_pos_[kinematics::MOTION_NUM];
        static double begin_joint_position_[kinematics::MOTION_NUM];

        static double cubic_coefs1_[4], cubic_coefs2_[4];
        static double pos2count_ratio_[kinematics::MOTION_NUM];
        static void calculate_coe(double* t, double* x, double* v, double* coe);
    };
}

#endif