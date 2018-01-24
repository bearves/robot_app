#ifndef ROBOT_DEFINITIONS_H
#define ROBOT_DEFINITIONS_H

namespace robot_app
{
    namespace kinematics
    {
        static const double PI = 3.1415926535898;
        static const int LEG_NUM    = 6;
        static const int MOTION_NUM = 13;
        static const int LEG_DOF    = 2;
        static const int WAIST_INDEX = 12;

        // Geometric parameters
        static const double L_AB = 190;
        static const double L_BE = 191.5;
        static const double THETA0[2] = {-35.8155900*PI/180.0, 35.4890058*PI/180.0};
    }
}

#endif
