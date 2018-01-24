#ifndef LEG_KINEMATICS_H 
#define LEG_KINEMATICS_H 

#include <cmath>

namespace robot_app
{
    namespace kinematics
    {
        class Leg
        {
        public:
            static void LegIK(double *tip_pos_in, double *joint_angle_out)
            {
                using namespace std;
                double len   = sqrt(tip_pos_in[0] * tip_pos_in[0] + tip_pos_in[1] * tip_pos_in[1]);
                double alpha = atan2(tip_pos_in[1], tip_pos_in[0]);
                double beta  = acos((L_AB * L_AB + len * len - L_BE * L_BE) / (2 * L_AB * len));
                double gamma = acos((L_AB * L_AB + L_BE * L_BE - len * len) / (2 * L_AB * L_BE));

                joint_angle_out[0] = alpha - beta + PI/2.0 - THETA0[0];
                joint_angle_out[1] = alpha - beta - gamma + PI + PI/2.0 - THETA0[1];
            }

            static void LegFK(double *joint_angle_in, double *tip_pos_out)
            {
                using namespace std;
                double theta0 = joint_angle_in[0] + THETA0[0] - PI/2.0;
                double theta1 = joint_angle_in[1] + THETA0[1] - PI/2.0;

                tip_pos_out[0] = L_AB * cos(theta0) + L_BE * cos(theta0);
                tip_pos_out[1] = L_AB * sin(theta0) + L_BE * sin(theta0);
            }
        };
    }
}

#endif
