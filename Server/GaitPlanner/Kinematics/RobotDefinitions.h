#ifndef ROBOT_DEFINITIONS_H
#define ROBOT_DEFINITIONS_H

namespace robot_app
{
    namespace kinematics
    {
        static const double PI = 3.141592653589793;
        static const int LEG_NUM        = 6;
        static const int ACTIVE_LEG_NUM = 6;
        static const int MOTION_NUM     = 13;
        static const int LEG_DOF        = 2;
        static const int WAIST_INDEX    = 12;
        static const int LEG_GROUP_A[3] = {0, 2, 4}; // MF, LB, RB
        static const int LEG_GROUP_B[3] = {1, 3, 5}; // LF, MB, RF

        // Geometric parameters for leg
        static const double L_AB = 0.300;
        static const double L_BE = 0.300;
        static const double THETA0[2] = {0, 0};

        // Geometric parameters for body

        // X axis : FRONT direction
        // Y axis : UP    direction
        // Z axis : RIGHT direction
        static const  double HIP_POSITION[3*LEG_NUM]=
            {  0.2525,   0.0, -0.25902,
               0.445  ,  0.0,        0,
               0.2525,   0.0,  0.25902,
              -0.2525,   0.0,  0.25902,
              -0.445  ,  0.0,        0,
              -0.2525,   0.0, -0.25902};        
        // orientations of each leg corresponding to the body frame
        // -1 means the leg's X and Z axis are opposite with the body's
        //  1 means the leg's X and Z axis are along with the body's
        static const double LEG_ORIENTATION[LEG_NUM] = {-1, -1, -1, 1, 1, 1};

        // Height at RC position
        static const double STANDING_HEIGHT = 0.45;
        static const double RETRACT_HEIGHT = 0.30*1.28;
        //stairclimb parament
        static const int TOTAL_CLIMB_NUMBER=23;
        static const int TOTAL_HILL_CLIMB_NUMBER=21;
	static const int TOTAL_CLIMB_UP_NUMBER=11;
        static const int TOTAL_DOWN_NUMBER=22;
    }
}

#endif
