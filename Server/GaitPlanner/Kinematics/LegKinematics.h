#ifndef LEG_KINEMATICS_H 
#define LEG_KINEMATICS_H 

#include <cmath>
#include "RobotDefinitions.h"

namespace robot_app
{
    namespace kinematics
    {
        class Leg
        {
        public:
            static void LegIK(double *tip_pos_in, double *joint_angle_out, double leg_orient);
            static void LegFK(double *joint_angle_in, double *tip_pos_out, double leg_orient);
        };
    }
}

#endif
