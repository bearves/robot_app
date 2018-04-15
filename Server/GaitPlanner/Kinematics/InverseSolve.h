#ifndef INVERSESOLVE_H
#define INVERSESOLVE_H
#include <iostream>
#include "LegKinematics.h"
#include <cmath>
#include "RobotDefinitions.h"
#include <Eigen/Dense>
#include <Eigen/Core>

namespace robot_app
{
    namespace kinematics
    {
        class O13Inverse
        {
        public:
            static void InverseSolve(double *tip_pos_in, double *body_pos_in, double *joint_angle_out);
        };
    }
}
#endif // INVERSESOLVE_H
