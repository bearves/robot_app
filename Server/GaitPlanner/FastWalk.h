#ifndef FAST_WALK_H
#define FAST_WALK_H

#include <aris.h>
#include <fstream>
#include "Kinematics/RobotDefinitions.h"
#include "Kinematics/LegKinematics.h"

void dlmread(const char *FileName, double *pMatrix);

namespace robot_app
{
    // for recovery
    struct FastWalkParam final : aris::server::GaitParamBase
    {
    public:
        double period;
        int step_number;
    };

    class FastWalk
    {
    public:
        static void setMotionSelector(const aris::server::MotionSelector &selector);
        static int  fastWalk(aris::model::Model &model, aris::server::PlanParamBase &param);
        static bool fastWalkParser(const std::string &cmd,
                               const std::map<std::string, std::string> &params,
                               aris::core::Msg &msg_out);

    private:
        static aris::server::MotionSelector motion_selector_;

        static int total_count_;
        static double Pee[6000][2];

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
