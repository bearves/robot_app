#ifndef INVERSESOLVELEG_H
#define INVERSESOLVELEG_H

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
        class O13InverseLeg
        {
        public:
            static void InverseSolveLeg13(double *tip_pos_in, double *body_pos_in, double *tip_pos_out);// inverse solve tip position in the leg coordination

        };
    }
}
#endif // INVERSESOLVE_H
