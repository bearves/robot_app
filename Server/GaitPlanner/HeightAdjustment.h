#ifndef HEIGHTADJUSTMENT_H
#define HEIGHTADJUSTMENT_H

#include <aris.h>
#include "Kinematics/RobotDefinitions.h"
#include "Kinematics/LegKinematics.h"
#include "Kinematics/InverseSolve.h"
#include "Kinematics/InverseSolveLeg.h"

namespace robot_app
{
    // for height adjustment
    struct HeightParam final : aris::server::GaitParamBase
    {
    public:
        double height_adjustment;
        double period;
    };

    class HeightAdjustment
    {
    public:
        static void setMotionSelector(const aris::server::MotionSelector &selector);
        static int  height(aris::model::Model &model, aris::server::PlanParamBase &param);
        static bool heightParser(const std::string &cmd,
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
        static double pos2count_ratio_[kinematics::MOTION_NUM];


        static double actual_height; //yk
        static double tip_[18];
        static double body[6];//body x\y\z\thetaz\thetay\thetax

    };
}


#endif // HEIGHTADJUSTMENT_H
