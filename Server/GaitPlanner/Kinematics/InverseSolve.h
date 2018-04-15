#ifndef INVERSESOLVE_H
#define INVERSESOLVE_H
#include <iostream>
#include "LegKinematics.h"
#include <cmath>
#include "RobotDefinitions.h"
#include <Eigen/Dense>
#include<Eigen/Core>

namespace robot_app
{
    namespace kinematics
    {
        class O13Inverse
        {
        public:
            static void InverseSolve(double *tip_pos_in, double *body_pos_in, double *joint_angle_out);
            //static void LegFK(double *joint_angle_in, double *tip_pos_out, double leg_orient);
        };
    }
}
#endif // INVERSESOLVE_H
