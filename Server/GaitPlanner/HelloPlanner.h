#ifndef HELLOPLANNER_H
#define HELLOPLANNER_H




#include <aris.h>
#include "Kinematics/RobotDefinitions.h"
#include "Kinematics/LegKinematics.h"
#include "Kinematics/InverseSolve.h"
#include "Kinematics/InverseSolveLeg.h"

namespace robot_app
{
    // for hello
    struct HelloParam final : aris::server::GaitParamBase
    {
    public:
        double alpha1;// swing zone
        double height_hello;// body position
        double beta1;// body prepare orient
        double period;// total time
        double x2;// swing position
        double y2;// swing position
        //double theta0[2];// swing prepare position
    };

    class HelloPlanner
    {
    public:
        static void setMotionSelector(const aris::server::MotionSelector &selector);
        static int  hello(aris::model::Model &model, aris::server::PlanParamBase &param);
        static bool helloParser(const std::string &cmd,
                               const std::map<std::string, std::string> &params,
                               aris::core::Msg &msg_out);

    private:
        static aris::server::MotionSelector motion_selector_;

        static int total_count_;
        static double begin_foot_pos_[18];
        static double begin_body_pos_[3];
        static double foot_pos_[18];
        static double body_pos_[3];
        static double tip_[18];
        static double body[6];//body x\y\z\thetaz\thetay\thetax
        static double begin_2_pos[12];// 2th phase motor xy position
        static double foot_cmd_pos_[kinematics::MOTION_NUM];
        static double joint_cmd_pos_[kinematics::MOTION_NUM];
        static double begin_joint_position_[kinematics::MOTION_NUM];
        static int total_count_part;
        static double pos2count_ratio_[kinematics::MOTION_NUM];
    };
}

#endif // HELLOPLANNER_H
