#ifndef STAIRCLIMB_PLANNER_H
#define STAIRCLIMB_PLANNER_H

#include <aris.h>
//#include "Kinematics/RobotDefinitions.h"
//#include "Kinematics/LegKinematics.h"
//#include "Kinematics/InverseSolve.h"
#include "Kinematics/InverseSolve.h"
#include <Eigen/Dense>

namespace robot_app
{
    // for recovery
    struct StairClimbParam final : aris::server::GaitParamBase
    {
    public:
        double step_length;
        double step_height;
        int step_number;
        double turning_rate;
        double period;
    };

    class StairClimbPlanner
    {
    public:
        static void setMotionSelector(const aris::server::MotionSelector &selector);
        static int  stairclimb(aris::model::Model &model, aris::server::PlanParamBase &param);
        static bool stairclimbParser(const std::string &cmd,
                               const std::map<std::string, std::string> &params, 
                               aris::core::Msg &msg_out);
    
    private:
        static aris::server::MotionSelector motion_selector_;

        static int total_count_;
        static double begin_foot_pos_[18];
        static double begin_body_pos_[6];
        static double foot_pos_[18];
        static double body_pos_[6];
        static double foot_cmd_pos_[kinematics::MOTION_NUM];
        static double joint_cmd_pos_[kinematics::MOTION_NUM];
        static double begin_joint_position_[kinematics::MOTION_NUM];
        static double foot_hold_key_points_[kinematics::TOTAL_CLIMB_NUMBER+1][kinematics::LEG_NUM*3];
        static double body_pos_key_points_[kinematics::TOTAL_CLIMB_NUMBER+1][6];
        static double distance_of_step_[kinematics::TOTAL_CLIMB_NUMBER][kinematics::LEG_NUM];
        static double height_of_step_[kinematics::TOTAL_CLIMB_NUMBER][kinematics::LEG_NUM];
        static double h2;
        static double distance_of_body_move_[kinematics::TOTAL_CLIMB_NUMBER];
        static double height_of_body_move_[kinematics::TOTAL_CLIMB_NUMBER];
        static double body_pitch_[kinematics::TOTAL_CLIMB_NUMBER];
        static double Delta_d_[kinematics::TOTAL_CLIMB_NUMBER];
        static double thetaMax;
        static int current_step_number;

        static double cubic_coefs1_[4], cubic_coefs2_[4];
        static double pos2count_ratio_[kinematics::MOTION_NUM];
        static void calculate_coe(double* t, double* x, double* v, double* coe);
        static void waistForwardKinematic(double &theta1, double &theta2, double &d);
        //static double waistForwardKinematic(double theta1,double theta2);
        //static void WalkPlanner::waistForwardKinematic(double theta1, double theta2, double d);
    };
    //void waistForwardKinematic1(double &theta1, double &theta2, double &d);
}

#endif
