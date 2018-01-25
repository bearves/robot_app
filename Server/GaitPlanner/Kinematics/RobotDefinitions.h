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
        static const int LEG_GROUP_A[3] = {0, 2, 4}; // MF, LB, RB
        static const int LEG_GROUP_B[3] = {1, 3, 5}; // LF, MB, RF

        // Geometric parameters for leg
        static const double L_AB = 0.190;
        static const double L_BE = 0.1915;
        static const double THETA0[2] = {-35.8155900*PI/180.0, 35.4890058*PI/180.0};

        // Geometric parameters for body

        // X axis : FRONT direction
        // Y axis : UP    direction
        // Z axis : RIGHT direction
        static const  double HIP_POSITION[3*LEG_NUM]=
            {  0.480  , 0.0,        0,
               0.23688, 0.0, -0.23688,
              -0.23688, 0.0, -0.23688,
              -0.480  , 0.0,        0,
              -0.23688, 0.0,  0.23688,
               0.23688, 0.0,  0.23688 };
        // orientations of each leg corresponding to the body frame
        // -1 means the leg's X and Z axis are opposite with the body's
        //  1 means the leg's X and Z axis are along with the body's
        static const double LEG_ORIENTATION[LEG_NUM] = {-1, -1, 1, 1, 1, -1};

        // Height at RC position
        static const double STANDING_HEIGHT = 0.31;
        static const double RETRACT_HEIGHT = 0.26;
    }
}

#endif
