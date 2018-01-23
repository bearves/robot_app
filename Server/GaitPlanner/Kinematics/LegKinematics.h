#ifndef LEG_KINEMATICS_H 
#define LEG_KINEMATICS_H 

namespace robot_app
{
    namespace kinematics
    {
        class Leg
        {
        public:
            static void LegIK(double *tip_pos_in, double *joint_angle_out)
            {
                for (int i = 0; i < LEG_DOF; i++)
                {
                    joint_angle_out[i] = tip_pos_in[i];
                }
            }
        };
    }
}

#endif