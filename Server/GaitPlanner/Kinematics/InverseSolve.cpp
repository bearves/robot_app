#include "InverseSolve.h"
namespace robot_app
{
    namespace kinematics
    {
        void O13Inverse::InverseSolve(double *tip_pos_in, double *body_pos_in, double *joint_angle_out)
        //void InverseSolve(double *tip_pos_in, double *body_pos_in,double *body_angle_in, double *joint_angle_out)
        {
          using namespace std;
            //test code
            // for (int i=0;i<6;i++)
            // {
            //     cout<<"foot pos on CS="<<tip_pos_in[3*i]<<" "<<tip_pos_in[3*i+1]<<" "<<tip_pos_in[3*i+2]<<endl;
            // }
            //     cout<<"body_pos_ pos on CS="<<body_pos_in[0]<<" "<<body_pos_in[1]<<" "<<body_pos_in[2]<<endl;
            //     cout<<"body_angle_ pos on CS="<<body_pos_in[3]<<" "<<body_pos_in[4]<<" "<<body_pos_in[5]<<endl;



            //void LegIK(double *tip_pos_in, double *joint_angle_out, double leg_orient);
            double beta=body_pos_in[3];
            double theta=body_pos_in[4];
            double alpha=body_pos_in[5];
            //zyx eular angle rotation matrix
            Eigen::Matrix4d Tz_beta;
            Eigen::Matrix4d Ty_theta;
            Eigen::Matrix4d Tx_alpha;
            Eigen::Matrix4d T;
            Eigen::Matrix4d T_inverse;

            Tz_beta<< cos(beta), -sin(beta), 0,  0,
                      sin(beta),  cos(beta), 0,  0,
                      0,         0,          1,  0,
                      0,         0,          0,  1;

            Ty_theta<<cos(theta),   0,  sin(theta), 0,
                      0,            1,  0,          0,
                      -sin(theta),  0,  cos(theta), 0,
                      0,            0,  0,          1;

            Tx_alpha<<1,   0,           0,           0,
                      0,   cos(alpha), -sin(alpha),  0,
                      0,   sin(alpha), cos(alpha),   0,
                      0,   0,           0,           1;
            T=Tz_beta*Ty_theta*Tx_alpha;

            T(0,3)=body_pos_in[0];
            T(1,3)=body_pos_in[1];
            T(2,3)=body_pos_in[2];
            T_inverse.topLeftCorner(3,3)=T.topLeftCorner(3,3).transpose();
            T_inverse.topRightCorner(3,1)=-T.topLeftCorner(3,3).transpose()*T.topRightCorner(3,1);
            T_inverse.bottomLeftCorner(1,4)<<0, 0,  0,  1;
            //Leg cs on body cs
            Eigen::Matrix4d T_b1_LFR;
            T_b1_LFR<<-1, 0, 0, 0.2520,
                       0, 1, 0, 0,
                       0, 0, -1,0.25902,
                       0, 0, 0, 1;
            Eigen::Matrix4d T_b1_LR;
            T_b1_LR<<  1, 0, 0, -0.445,
                       0, 1, 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1;
            Eigen::Matrix4d T_b1_LFL;
            T_b1_LFL<<-1, 0, 0, 0.2520,
                       0, 1, 0, 0,
                       0, 0, -1,-0.25902,
                       0, 0, 0, 1;
            Eigen::Matrix4d T_b2_LF;
            T_b2_LF<<  -1, 0,  0, 0.445,
                        0, 1,  0, 0,
                        0, 0, -1, 0,
                        0, 0,  0, 1;
            Eigen::Matrix4d T_b2_LRR;
            T_b2_LRR<< 1, 0, 0, -0.2520,
                       0, 1, 0, 0,
                       0, 0, 1, 0.25902,
                       0, 0, 0, 1;
            Eigen::Matrix4d T_b2_LRL;
            T_b2_LRL<< 1, 0, 0, -0.2520,
                       0, 1, 0, 0,
                       0, 0, 1, -0.25902,
                       0, 0, 0, 1;
            //body cs on Leg cs
            Eigen::Matrix4d T_LFR_b1;
            //T_LFR_b1=T_b1_LFR.inverse();
            T_LFR_b1.topLeftCorner(3,3)=T_b1_LFR.topLeftCorner(3,3).transpose();
            T_LFR_b1.topRightCorner(3,1)=-T_b1_LFR.topLeftCorner(3,3).transpose()*T_b1_LFR.topRightCorner(3,1);
            T_LFR_b1.bottomLeftCorner(1,4)<<0, 0,  0,  1;

            Eigen::Matrix4d T_LR_b1;
            //T_LR_b1=T_b1_LR.inverse();
            T_LR_b1.topLeftCorner(3,3)=T_b1_LR.topLeftCorner(3,3).transpose();
            T_LR_b1.topRightCorner(3,1)=-T_b1_LR.topLeftCorner(3,3).transpose()*T_b1_LR.topRightCorner(3,1);
            T_LR_b1.bottomLeftCorner(1,4)<<0, 0,  0,  1;

            Eigen::Matrix4d T_LFL_b1;
            //T_LFL_b1=T_b1_LFL.inverse();
            T_LFL_b1.topLeftCorner(3,3)=T_b1_LFL.topLeftCorner(3,3).transpose();
            T_LFL_b1.topRightCorner(3,1)=-T_b1_LFL.topLeftCorner(3,3).transpose()*T_b1_LFL.topRightCorner(3,1);
            T_LFL_b1.bottomLeftCorner(1,4)<<0, 0,  0,  1;

            Eigen::Matrix4d T_LF_b2;
            //T_LF_b2=T_b2_LF.inverse();
            T_LF_b2.topLeftCorner(3,3)=T_b2_LF.topLeftCorner(3,3).transpose();
            T_LF_b2.topRightCorner(3,1)=-T_b2_LF.topLeftCorner(3,3).transpose()*T_b2_LF.topRightCorner(3,1);
            T_LF_b2.bottomLeftCorner(1,4)<<0, 0,  0,  1;

            Eigen::Matrix4d T_LRR_b2;
            //T_LRR_b2=T_b2_LRR.inverse();
            T_LRR_b2.topLeftCorner(3,3)=T_b2_LRR.topLeftCorner(3,3).transpose();
            T_LRR_b2.topRightCorner(3,1)=-T_b2_LRR.topLeftCorner(3,3).transpose()*T_b2_LRR.topRightCorner(3,1);
            T_LRR_b2.bottomLeftCorner(1,4)<<0, 0,  0,  1;

            Eigen::Matrix4d T_LRL_b2;
            //T_LRL_b2=T_b2_LRL.inverse();
            T_LRL_b2.topLeftCorner(3,3)=T_b2_LRL.topLeftCorner(3,3).transpose();
            T_LRL_b2.topRightCorner(3,1)=-T_b2_LRL.topLeftCorner(3,3).transpose()*T_b2_LRL.topRightCorner(3,1);
            T_LRL_b2.bottomLeftCorner(1,4)<<0, 0,  0,  1;            

            //tip on GCS
            Eigen::Vector4d Q_FF;
            Q_FF<<tip_pos_in[0],tip_pos_in[1],tip_pos_in[2],1;
            Eigen::Vector4d Q_FFL;
            Q_FFL<<tip_pos_in[3],tip_pos_in[4],tip_pos_in[5],1;
            Eigen::Vector4d Q_FRL;
            Q_FRL<<tip_pos_in[6],tip_pos_in[7],tip_pos_in[8],1;
            Eigen::Vector4d Q_FR;
            Q_FR<<tip_pos_in[9],tip_pos_in[10],tip_pos_in[11],1;
            Eigen::Vector4d Q_FRR;
            Q_FRR<<tip_pos_in[12],tip_pos_in[13],tip_pos_in[14],1;
            Eigen::Vector4d Q_FFR;
            Q_FFR<<tip_pos_in[15],tip_pos_in[16],tip_pos_in[17],1;
            //tip on LEG CS
            Eigen::Vector4d QL_FF;
            Eigen::Vector4d QL_FFL;
            Eigen::Vector4d QL_FRL;
            Eigen::Vector4d QL_FR;
            Eigen::Vector4d QL_FRR;
            Eigen::Vector4d QL_FFR;
            QL_FF=T_LF_b2*T_inverse*Q_FF;
            QL_FFL=T_LFL_b1*T_inverse*Q_FFL;
            QL_FRL=T_LRL_b2*T_inverse*Q_FRL;
            QL_FR=T_LR_b1*T_inverse*Q_FR;
            QL_FRR=T_LRR_b2*T_inverse*Q_FRR;
            QL_FFR=T_LFR_b1*T_inverse*Q_FFR;
            // std::cout<<"T_B_G ="<<T<<endl;
            // std::cout<<"T_inverse ="<<T_inverse<<endl;
            // std::cout<<"T_LF_B2 ="<<T_LF_b2<<endl;
            // std::cout<<"T_LFL_B1 ="<<T_LFL_b1<<endl;
            // std::cout<<"T_LRL_B2 ="<<T_LRL_b2<<endl;
            // std::cout<<"T_LR_B1 ="<<T_LR_b1<<endl;
            // std::cout<<"T_LRR_B2 ="<<T_LRR_b2<<endl;
            // std::cout<<"T_LFR_B1 ="<<T_LFR_b1<<endl;

            // std::cout<<"foot FF position ON GCS ="<<Q_FF<<endl;
            // std::cout<<"foot FF1 position ON LCS="<<QL_FF<<endl;
            // std::cout<<"foot FF2 position ON LCS="<<QL_FFL<<endl;
            // std::cout<<"foot FF3 position ON LCS="<<QL_FRL<<endl;
            // std::cout<<"foot FF4 position ON LCS="<<QL_FR<<endl;
            // std::cout<<"foot FF5 position ON LCS="<<QL_FRR<<endl;
            // std::cout<<"foot FF5 position ON LCS="<<QL_FFR<<endl;

            //********moter angle by using legik*******void LegIK(double *tip_pos_in, double *joint_angle_out, double leg_orient);
            // double TipPos[12]{QL_FF(0),QL_FF(1),
            //                   QL_FFL(0),QL_FFL(1),
            //                   QL_FRL(0),QL_FRL(1),
            //                   QL_FR(0),QL_FR(1),
            //                   QL_FRR(0),QL_FRR(1),
            //                   QL_FFR(0),QL_FFR(1)
            //                   };
            double TipPos[12]{QL_FFL(0),QL_FFL(1),
                              QL_FF(0),QL_FF(1),
                              QL_FFR(0),QL_FFR(1),
                              QL_FRR(0),QL_FRR(1),
                              QL_FR(0),QL_FR(1),
                              QL_FRL(0),QL_FRL(1)
                              };                              

            for (int i=0;i<6;i++)
            {

               kinematics::Leg::LegIK(
                   &(TipPos[i*2]),
                   &(joint_angle_out[i*2]),
                     1);
            }



        }
     }

}
