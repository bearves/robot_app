#include "DownStairPlanner.h"
#include <cmath>
#include <iostream>

namespace robot_app
{
    aris::server::MotionSelector DownStairPlanner::motion_selector_ = nullptr;
    int DownStairPlanner::total_count_;
    double DownStairPlanner::begin_foot_pos_[18];
    double DownStairPlanner::begin_body_pos_[6];
    double DownStairPlanner::foot_pos_[18];
    double DownStairPlanner::body_pos_[6];
    double DownStairPlanner::begin_joint_position_[kinematics::MOTION_NUM];
    double DownStairPlanner::foot_cmd_pos_[kinematics::MOTION_NUM];
    double DownStairPlanner::joint_cmd_pos_[kinematics::MOTION_NUM];
    double DownStairPlanner::pos2count_ratio_[kinematics::MOTION_NUM];
    double DownStairPlanner::cubic_coefs1_[4];
    double DownStairPlanner::cubic_coefs2_[4];
    double DownStairPlanner::thetaMax;
    double DownStairPlanner::foot_hold_key_points_[kinematics::TOTAL_DOWN_NUMBER+1][kinematics::LEG_NUM*3]
                                                   {
        2.4000 ,0.5300 ,0.0000 ,2.2075 ,0.5300 ,0.2590 ,1.7025 ,0.5300 ,-0.2590 ,1.5100 ,0.5300 ,0.0000 ,1.7025 ,0.5300 ,0.2590 ,2.2075 ,0.5300 ,0.2590 ,
        2.1800 ,0.5300 ,0.0000 ,2.2075 ,0.5300 ,0.2590 ,1.5100 ,0.5300 ,-0.2590 ,1.5100 ,0.5300 ,0.0000 ,1.5100 ,0.5300 ,0.2590 ,2.2075 ,0.5300 ,0.2590 ,
        2.1800 ,0.5300 ,0.0000 ,1.9600 ,0.5300 ,0.2590 ,1.5100 ,0.5300 ,-0.2590 ,1.2900 ,0.3600 ,0.0000 ,1.5100 ,0.5300 ,0.2590 ,1.9600 ,0.5300 ,0.2590 ,
        1.9600 ,0.5300 ,0.0000 ,1.9600 ,0.5300 ,0.2590 ,1.2900 ,0.3600 ,-0.2590 ,1.2900 ,0.3600 ,0.0000 ,1.2900 ,0.3600 ,0.2590 ,1.9600 ,0.5300 ,0.2590 ,
        1.9600 ,0.5300 ,0.0000 ,1.7100 ,0.5300 ,0.2590 ,1.2900 ,0.3600 ,-0.2590 ,1.1300 ,0.3600 ,0.0000 ,1.2900 ,0.3600 ,0.2590 ,1.7100 ,0.5300 ,0.2590 ,
        1.7100 ,0.5300 ,0.0000 ,1.7100 ,0.5300 ,0.2590 ,1.1300 ,0.3600 ,-0.2590 ,1.1300 ,0.3600 ,0.0000 ,1.1300 ,0.3600 ,0.2590 ,1.7100 ,0.5300 ,0.2590 ,
        1.7100 ,0.5300 ,0.0000 ,1.6000 ,0.5300 ,0.2590 ,1.1300 ,0.3600 ,-0.2590 ,0.9500 ,0.1900 ,0.0000 ,1.1300 ,0.3600 ,0.2590 ,1.6000 ,0.5300 ,0.2590 ,
        1.6000 ,0.5300 ,0.0000 ,1.6000 ,0.5300 ,0.2590 ,0.9500 ,0.1900 ,-0.2590 ,0.9500 ,0.1900 ,0.0000 ,0.9500 ,0.1900 ,0.2590 ,1.6000 ,0.5300 ,0.2590 ,
        1.6000 ,0.5300 ,0.0000 ,1.5000 ,0.5300 ,0.2590 ,0.9500 ,0.1900 ,-0.2590 ,0.8200 ,0.1900 ,0.0000 ,0.9500 ,0.1900 ,0.2590 ,1.5000 ,0.5300 ,0.2590 ,
        1.5000 ,0.5300 ,0.0000 ,1.5000 ,0.5300 ,0.2590 ,0.8200 ,0.1900 ,-0.2590 ,0.8200 ,0.1900 ,0.0000 ,0.8200 ,0.1900 ,0.2590 ,1.5000 ,0.5300 ,0.2590 ,
        1.5000 ,0.5300 ,0.0000 ,1.3200 ,0.3600 ,0.2590 ,0.8200 ,0.1900 ,-0.2590 ,0.7600 ,0.1900 ,0.0000 ,0.8200 ,0.1900 ,0.2590 ,1.3200 ,0.3600 ,0.2590 ,
        1.3200 ,0.3600 ,0.0000 ,1.3200 ,0.3600 ,0.2590 ,0.7600 ,0.1900 ,-0.2590 ,0.7600 ,0.1900 ,0.0000 ,0.7600 ,0.1900 ,0.2590 ,1.3200 ,0.3600 ,0.2590 ,
        1.3200 ,0.3600 ,0.0000 ,1.2300 ,0.3600 ,0.2590 ,0.7600 ,0.1900 ,-0.2590 ,0.5800 ,0.0200 ,0.0000 ,0.7600 ,0.1900 ,0.2590 ,1.2300 ,0.3600 ,0.2590 ,
        1.2300 ,0.3600 ,0.0000 ,1.2300 ,0.3600 ,0.2590 ,0.5800 ,0.0200 ,-0.2590 ,0.5800 ,0.0200 ,0.0000 ,0.5800 ,0.0200 ,0.2590 ,1.2300 ,0.3600 ,0.2590 ,
        1.2300 ,0.3600 ,0.0000 ,1.1300 ,0.3600 ,0.2590 ,0.5800 ,0.0200 ,-0.2590 ,0.4300 ,0.0200 ,0.0000 ,0.5800 ,0.0200 ,0.2590 ,1.1300 ,0.3600 ,0.2590 ,
        1.1300 ,0.3600 ,0.0000 ,1.1300 ,0.3600 ,0.2590 ,0.4300 ,0.0200 ,-0.2590 ,0.4300 ,0.0200 ,0.0000 ,0.4300 ,0.0200 ,0.2590 ,1.1300 ,0.3600 ,0.2590 ,
        1.1300 ,0.3600 ,0.0000 ,0.9500 ,0.1900 ,0.2590 ,0.4300 ,0.0200 ,-0.2590 ,0.2900 ,0.0200 ,0.0000 ,0.4300 ,0.0200 ,0.2590 ,0.9500 ,0.1900 ,0.2590 ,
        0.9500 ,0.1900 ,0.0000 ,0.9500 ,0.1900 ,0.2590 ,0.2900 ,0.0200 ,-0.2590 ,0.2900 ,0.0200 ,0.0000 ,0.2900 ,0.0200 ,0.2590 ,0.9500 ,0.1900 ,0.2590 ,
        0.9500 ,0.1900 ,0.0000 ,0.7600 ,0.1900 ,0.2590 ,0.2900 ,0.0200 ,-0.2590 ,0.0500 ,0.0200 ,0.0000 ,0.2900 ,0.0200 ,0.2590 ,0.7600 ,0.1900 ,0.2590 ,
        0.7600 ,0.1900 ,0.0000 ,0.7600 ,0.1900 ,0.2590 ,0.0500 ,0.0200 ,-0.2590 ,0.0500 ,0.0200 ,0.0000 ,0.0500 ,0.0200 ,0.2590 ,0.7600 ,0.1900 ,0.2590 ,
        0.7600 ,0.1900 ,0.0000 ,0.4400 ,0.0200 ,0.2590 ,0.0500 ,0.0200 ,-0.2590 ,-0.2600 ,0.0200 ,0.0000 ,0.0500 ,0.0200 ,0.2590 ,0.4400 ,0.0200 ,0.2590 ,
        0.4450 ,0.0200 ,0.0000 ,0.4400 ,0.0200 ,0.2590 ,-0.2525 ,0.0200 ,-0.2590 ,-0.2600 ,0.0200 ,0.0000 ,-0.2525 ,0.0200 ,0.2590 ,0.4400 ,0.0200 ,0.2590 ,
        0.4450 ,0.0200 ,0.0000 ,0.2525 ,0.0200 ,0.2590 ,-0.2525 ,0.0200 ,-0.2590 ,-0.4450 ,0.0200 ,0.0000 ,-0.2525 ,0.0200 ,0.2590 ,0.2525 ,0.0200 ,0.2590 ,
                                                   };
    double DownStairPlanner::body_pos_key_points_[kinematics::TOTAL_DOWN_NUMBER+1][6]
                                                    {
                                                        1.9550 ,0.9800 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,
                                                        1.8450 ,0.9700 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,
                                                        1.7332 ,0.9396 ,0.0000 ,0.2485 ,0.0000 ,0.0000 ,
                                                        1.5159 ,0.8749 ,0.0000 ,0.2485 ,0.0000 ,0.0000 ,
                                                        1.5135 ,0.8846 ,0.0000 ,0.2485 ,0.0000 ,0.0000 ,
                                                        1.2964 ,0.8669 ,0.0000 ,0.2851 ,0.0000 ,0.0000 ,
                                                        1.2953 ,0.8565 ,0.0000 ,0.2851 ,0.0000 ,0.0000 ,
                                                        1.0763 ,0.7398 ,0.0000 ,0.4819 ,0.0000 ,0.0000 ,
                                                        1.0763 ,0.7398 ,0.0000 ,0.4819 ,0.0000 ,0.0000 ,
                                                        0.9674 ,0.7451 ,0.0000 ,0.4636 ,0.0000 ,0.0000 ,
                                                        0.9674 ,0.7451 ,0.0000 ,0.4636 ,0.0000 ,0.0000 ,
                                                        0.8226 ,0.6947 ,0.0000 ,0.2947 ,0.0000 ,0.0000 ,
                                                        0.8216 ,0.6843 ,0.0000 ,0.3209 ,0.0000 ,0.0000 ,
                                                        0.7063 ,0.5698 ,0.0000 ,0.4819 ,0.0000 ,0.0000 ,
                                                        0.7063 ,0.5698 ,0.0000 ,0.4819 ,0.0000 ,0.0000 ,
                                                        0.5913 ,0.5784 ,0.0000 ,0.4522 ,0.0000 ,0.0000 ,
                                                        0.5913 ,0.5784 ,0.0000 ,0.4522 ,0.0000 ,0.0000 ,
                                                        0.4395 ,0.5341 ,0.0000 ,0.2521 ,0.0000 ,0.0000 ,
                                                        0.4470 ,0.5438 ,0.0000 ,0.2521 ,0.0000 ,0.0000 ,
                                                        0.3014 ,0.5377 ,0.0000 ,0.2350 ,0.0000 ,0.0000 ,
                                                        0.2944 ,0.5078 ,0.0000 ,0.2350 ,0.0000 ,0.0000 ,
                                                        0.0702 ,0.4700 ,0.0000 ,0.0500 ,0.0000 ,0.0000 ,
                                                        0.0000 ,0.4700 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,
                                                     };
    double DownStairPlanner::distance_of_step_[kinematics::TOTAL_DOWN_NUMBER][kinematics::LEG_NUM];
    double DownStairPlanner::height_of_step_[kinematics::TOTAL_DOWN_NUMBER][kinematics::LEG_NUM];
    double DownStairPlanner::h2=0.040;
    double DownStairPlanner::distance_of_body_move_[kinematics::TOTAL_DOWN_NUMBER];
    double DownStairPlanner::height_of_body_move_[kinematics::TOTAL_DOWN_NUMBER];
    double DownStairPlanner::body_pitch_[kinematics::TOTAL_DOWN_NUMBER];
    int    DownStairPlanner::current_step_number;
    double DownStairPlanner::Delta_d_[kinematics::TOTAL_DOWN_NUMBER]{0};
    void DownStairPlanner::setMotionSelector(const aris::server::MotionSelector &selector)
    {
        motion_selector_ = selector;
    }

    int DownStairPlanner::downstair(aris::model::Model &model, aris::server::PlanParamBase &param)
    {
        using aris::control::EthercatMotion;
        using kinematics::PI;

        auto& sc_param = static_cast<DownStairParam &>(param);
        static aris::server::ControlServer &cs = aris::server::ControlServer::instance();
        /////////////////////////////////////////////////////////////
        // calculate step_h and step_d Matrix here                        //
        /////////////////////////////////////////////////////////////
        if (sc_param.count == 0)
        {
            for (int i=0; i<kinematics::LEG_NUM; i++)
            {
                //std::cout<<"key point "<<std::endl;
                for(int j=0;j<kinematics::TOTAL_DOWN_NUMBER+1;j++)
                {
                    //std::cout<<foot_hold_key_points_[j][3*i]<<" "<<foot_hold_key_points_[j][3*i+1]<<" "<<foot_hold_key_points_[j][3*i+2]<<" ";
                }
            }  
            for (int i=0; i<6; i++)
            {
                // std::cout<<"total down number = "<<kinematics::TOTAL_DOWN_NUMBER<<std::endl;
                // std::cout<<std::endl<<"key body point"<<std::endl;
                for(int j=0;j<kinematics::TOTAL_DOWN_NUMBER+1;j++)
                {
                    // std::cout<<body_pos_key_points_[j][i+1]<<" ";

                }
            }               


            for (int i=0; i<kinematics::LEG_NUM; i++)
            {
                for(int j=0;j<kinematics::TOTAL_DOWN_NUMBER;j++)
                {
                    distance_of_step_[j][i]=foot_hold_key_points_[j+1][i*3]-foot_hold_key_points_[j][i*3];
                    height_of_step_[j][i]=foot_hold_key_points_[j+1][i*3+1]-foot_hold_key_points_[j][i*3+1];
                }
            }

            // for (int j=0;j<kinematics::TOTAL_DOWN_NUMBER;j++)
            // {
            //     std::cout<<std::endl<<"distance_of_step_=";
            //     for(int i=0; i<kinematics::LEG_NUM; i++)
            //     {
            //         std::cout<<distance_of_step_[j][i]<<std::endl;
            //     }
            // }
            for (int i=0;i<kinematics::TOTAL_DOWN_NUMBER;i++)
            {
                distance_of_body_move_[i]=body_pos_key_points_[i+1][0]-body_pos_key_points_[i][0];
                height_of_body_move_[i]=body_pos_key_points_[i+1][1]-body_pos_key_points_[i][1];
                body_pitch_[i]=body_pos_key_points_[i+1][3]-body_pos_key_points_[i][3];
            }

                // std::cout<<std::endl<<"distance_of_body_move_=";
                // for(int i=0; i<6; i++)
                // {
                //     std::cout<<distance_of_body_move_[i]<<std::endl;
                //     std::cout<<body_pitch_[i]<<std::endl;
                // }
        }


        /////////////////////////////////////////////////////////////
        // Initialize beginning states here                        //
        /////////////////////////////////////////////////////////////
        //  if (sc_param.count == 201)
        //  {
        //      while(1)
        //      {}
        //  }


        if (sc_param.count == 0)
        {
            // get pos2count ratio and begin joint position for each motor
            for (int i = 0; i < kinematics::MOTION_NUM; i++)
            {
                pos2count_ratio_[i] = cs.controller().motionAtAbs(i).pos2countRatio();
                begin_joint_position_[i] = (*sc_param.motion_raw_data)[i].feedback_pos / pos2count_ratio_[i];
            }

            // calculate cubic coefficients for body trj interpolation
            total_count_ = (int)(sc_param.period * cs.getControlFreq());


            // set body's begin pos
            std::fill(begin_body_pos_, begin_body_pos_+6, 0);

            // set leg's begin pos
            for (int i = 0; i < kinematics::ACTIVE_LEG_NUM; i++)
            {
                kinematics::Leg::LegFK(
                    &(begin_joint_position_[i*2]), 
                    &(begin_foot_pos_[i*3]), 
                    kinematics::LEG_ORIENTATION[i]);
                if (i==0||i==3)
                {
                    begin_foot_pos_[i*3+2] = 0;
                }
                else if(i==1||i==2)
                {
                    begin_foot_pos_[i*3+2] = -0.25902;
                }
                else
                {
                    begin_foot_pos_[i*3+2] = 0.25902;
                }

                // rt_printf("Leg %d's begin position is (%.3f, %.3f)\n", 
                //     i,
                //     begin_foot_pos_[i*3], 
                //     begin_foot_pos_[i*3+1]);
            }
            std::copy(begin_foot_pos_, begin_foot_pos_+18, foot_pos_);
            std::copy(begin_body_pos_, begin_body_pos_+6,  body_pos_);
            // std::cout<<std::endl<<"begin_joint_position_=";
            // for (int i=0;i<12;i++)
            // {
            //     std::cout<<begin_joint_position_[i]<<" ";
            // }
            // std::cout<<std::endl<<"begin_foot_pos_=";
            // for (int i=0;i<18;i++)
            // {
            //     std::cout<<begin_foot_pos_[i]<<" ";
            // }

            // std::cout<<std::endl<<"begin_body_pos_="<<" ";
            // for (int i=0;i<6;i++)
            // {
            //     std::cout<<begin_body_pos_[i];
            // }

            
        }

        ///////////////////////////////////////////////////////////////////
        // User Planning code begins here                                //
        // You need to calculate joint_cmd_pos_ in this part             //
        ///////////////////////////////////////////////////////////////////
        int period_count = sc_param.count % total_count_;
        double s = -PI / 2.0 * std::cos( PI * (period_count+1) *1.0 / total_count_) + PI / 2.0;
        double s1 = -PI / 2.0 * std::cos( PI * (period_count-0.1*total_count_ + 1) *1.0 / (total_count_-0.2*total_count_)) + PI / 2.0;
        DownStairPlanner::current_step_number=sc_param.count/total_count_;
        if (period_count == 0)
        // if (sc_param.count == 0)
        {
            // std::copy(foot_pos_, foot_pos_+18, begin_foot_pos_);
            // std::copy(body_pos_, body_pos_+6,  begin_body_pos_);
            if(current_step_number==0)
            {
             std::copy(*foot_hold_key_points_+current_step_number*18, *foot_hold_key_points_+(current_step_number+1)*18, begin_foot_pos_);
             std::copy(*body_pos_key_points_+current_step_number*6, *body_pos_key_points_+(current_step_number+1)*6,  begin_body_pos_);
            }
            else
            {
             std::copy(*foot_hold_key_points_+current_step_number*18, *foot_hold_key_points_+(current_step_number+1)*18, begin_foot_pos_);
             std::copy(*body_pos_key_points_+current_step_number*6, *body_pos_key_points_+(current_step_number+1)*6,  begin_body_pos_);
            
            //  std::copy(foot_pos_, foot_pos_+18, begin_foot_pos_);
            //  std::copy(body_pos_, body_pos_+6,  begin_body_pos_);
            }

             // for (int i=0;i<18;i++)
            // {
            //    begin_foot_pos_[i]= foot_hold_key_points_[1][i];
            // }
            // for (int i=0;i<6;i++)
            // {
            //    begin_body_pos_[i]= body_pos_key_points_[1][i];
            // }

            //  std::cout<<"begin_foot_pos=";

            //  for (int i=0;i<18;i++)
            //  {
            //      std::cout<<*(begin_foot_pos_+i)<<" ";
            //  }
            //  std::cout<<std::endl;
        }
        
        double current_step_len;
        const int* swing_leg_group;
        const int* stance_leg_group;
        double t_r = (period_count + 1) * 1.0 / cs.getControlFreq();
        double t_r2 = t_r * t_r;
        double t_r3 = t_r2 * t_r;
        double waist_BeginTurnAngle;
        double waist_EndTurnAngle;
        double waist_CurrentTurnAngle_GroupA;
        double waist_CurrentTurnAngle_GroupB;
        double waistScrewLenth;
        //waist planning
        if((int)(sc_param.count/total_count_)%2==0)
        {
            //leg group A swing and waist planning
            waist_BeginTurnAngle=0*kinematics::PI/180.0;
            waist_EndTurnAngle=sc_param.turning_rate*kinematics::PI/180.0;
            if (period_count<0.1*total_count_)
            {
               waist_CurrentTurnAngle_GroupA=waist_BeginTurnAngle;
               waist_CurrentTurnAngle_GroupB=waist_BeginTurnAngle;
            }
            else if(period_count>0.9*total_count_)
            {
               waist_CurrentTurnAngle_GroupA=waist_EndTurnAngle;
            }
            else
            {
               waist_CurrentTurnAngle_GroupA=0.5*(waist_BeginTurnAngle+waist_EndTurnAngle)-0.5*(waist_BeginTurnAngle-waist_EndTurnAngle)*std::cos(s1);
            }


        }
        else
        {
            //leg group B swing and waist planning
            if (period_count<0.1*total_count_)
            {
               waist_CurrentTurnAngle_GroupB=waist_BeginTurnAngle;
            }
            else if(period_count>0.9*total_count_)
            {
               waist_CurrentTurnAngle_GroupB=waist_EndTurnAngle;
            }
            else
            {
               waist_CurrentTurnAngle_GroupB=0.5*(waist_BeginTurnAngle+waist_EndTurnAngle)-0.5*(waist_BeginTurnAngle-waist_EndTurnAngle)*std::cos(s1);
            }
        }

        // trajectory planning
        //my trajectory planning code begin
        // DownStairPlanner::current_step_number=sc_param.count/total_count_;

           //body traj generate
           body_pos_[0]=begin_body_pos_[0]
                   +(distance_of_body_move_[DownStairPlanner::current_step_number]-0.5*DownStairPlanner::Delta_d_[DownStairPlanner::current_step_number])
                   *(1-std::cos(s))/2.0;
           body_pos_[1]=begin_body_pos_[1]+height_of_body_move_[DownStairPlanner::current_step_number]*(1-std::cos(s))/2.0;
           body_pos_[3]=begin_body_pos_[3]+body_pitch_[DownStairPlanner::current_step_number]*(1-std::cos(s))/2.0;

           //tip traj generate
           for (int i=0;i<6;i++)
           {
               int leg_id=i;
               foot_pos_[leg_id*3+0] = begin_foot_pos_[leg_id*3+0]
                       + (DownStairPlanner::distance_of_step_[DownStairPlanner::current_step_number][leg_id]-DownStairPlanner::Delta_d_[DownStairPlanner::current_step_number])
                       *(1-std::cos(s))/2.0;
               if(DownStairPlanner::distance_of_step_[DownStairPlanner::current_step_number][leg_id]==0)
               {
                   h2=0;
               }
               else
               {
                   h2=0.040;
               }
               if(period_count<total_count_/2)
               {
                   if(DownStairPlanner::height_of_step_[DownStairPlanner::current_step_number][leg_id]>=0)
                   {
                       foot_pos_[leg_id*3+1] = begin_foot_pos_[leg_id*3+1]
                               + (DownStairPlanner::height_of_step_[DownStairPlanner::current_step_number][leg_id]+h2) * std::sin(s);
                   }
                   else
                   {
                       foot_pos_[leg_id*3+1] = begin_foot_pos_[leg_id*3+1]
                               + h2 * std::sin(s);
                   }

               }
               else
               {
                   if(DownStairPlanner::height_of_step_[DownStairPlanner::current_step_number][leg_id]>=0)
                   {
                       foot_pos_[leg_id*3+1] = begin_foot_pos_[leg_id*3+1]
                               +DownStairPlanner::height_of_step_[DownStairPlanner::current_step_number][leg_id]+h2*std::sin(s);
                   }
                   else
                   {
                       foot_pos_[leg_id*3+1] = begin_foot_pos_[leg_id*3+1]
                               +DownStairPlanner::height_of_step_[DownStairPlanner::current_step_number][leg_id]
                               +(h2-DownStairPlanner::height_of_step_[DownStairPlanner::current_step_number][leg_id])*std::sin(s);
                   }

               }
               foot_pos_[leg_id*3+2] = begin_foot_pos_[leg_id*3+2];
           }

        if ((sc_param.count-3)%4000 == 0 || sc_param.count==3999)
        {
            std::cout<<"footPos in count="<<sc_param.count<<std::endl;
            std::cout<<" s ="<< s <<std::endl;
            for (int i=0;i<2;i++)
            {
                std::cout<<foot_pos_[i]<<" ";
            }
            std::cout<<std::endl;

            std::cout<<"bodyPos in count="<<sc_param.count<<std::endl;
            for (int i=0;i<6;i++)
            {
                std::cout<<body_pos_[i]<<" ";
            }
            std::cout<<std::endl;
        }

        // if (sc_param.count == 4000)
        // {
        //     std::cout<<"footPos in 4000 count=";
        //     for (int i=0;i<18;i++)
        //     {
        //         std::cout<<foot_pos_[i]<<" ";
        //     }
        //     std::cout<<std::endl;

        //     std::cout<<"bodyPos in 4000 count=";
        //     for (int i=0;i<6;i++)
        //     {
        //         std::cout<<body_pos_[i]<<" ";
        //     }
        //     std::cout<<std::endl;
        // }

        // if (sc_param.count == 3999)
        // {
        //     std::cout<<"footPos in 3999 count="<<std::endl;
        //     for (int i=0;i<18;i++)
        //     {
        //         std::cout<<foot_pos_[i]<<" ";
        //     }
        //     std::cout<<std::endl;

        //     std::cout<<"bodyPos in 3999 count="<<std::endl;
        //     for (int i=0;i<6;i++)
        //     {
        //         std::cout<<body_pos_[i]<<" ";
        //     }
        //     std::cout<<std::endl;
        // }

        


        // Leg joint cmd
        kinematics::O13Inverse::InverseSolve( foot_pos_,body_pos_,joint_cmd_pos_);

        if ((sc_param.count-3)%400 == 0|| sc_param.count==3999)
        {
            std::cout<<"foot pos in in count="<<sc_param.count<<std::endl;
            for (int i=0;i<18;i++)
            {
                std::cout<<foot_pos_[i]<<" ";
            }

            std::cout<<"body pos in count="<<sc_param.count<<std::endl;
            for (int i=0;i<6;i++)
            {
                std::cout<<body_pos_[i]<<" ";
            }

            std::cout<<"angle out in count="<<sc_param.count<<std::endl;
            for (int i=0;i<12;i++)
            {
                std::cout<<joint_cmd_pos_[i]<<" ";
            }
            std::cout<<std::endl;
        }


        // Waist joint cmd
        joint_cmd_pos_[kinematics::WAIST_INDEX] = begin_joint_position_[kinematics::WAIST_INDEX];
        // waistForwardKinematic(
        //                       waist_CurrentTurnAngle_GroupB,
        //                       waist_CurrentTurnAngle_GroupA,
        //                       waistScrewLenth);
        // joint_cmd_pos_[kinematics::WAIST_INDEX]=waistScrewLenth;

        ///////////////////////////////////////////////////////
        // User Planning code ends here                      //
        ///////////////////////////////////////////////////////

        // don't change the following part unless you DO know what you are doing
        for (int i = 0; i < kinematics::MOTION_NUM; i++)
        {
            if (sc_param.active_motor[i])
            {
                (*sc_param.motion_raw_data)[i].cmd = EthercatMotion::Cmd::RUN;
                (*sc_param.motion_raw_data)[i].mode = EthercatMotion::Mode::POSITION;
                (*sc_param.motion_raw_data)[i].target_pos = joint_cmd_pos_[i] * pos2count_ratio_[i];
            }
        }

        // return 0 if planning finished, otherwise return positive value
        return  kinematics::TOTAL_DOWN_NUMBER * total_count_ - sc_param.count -1;
    }

    bool DownStairPlanner::downstairParser(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)
    {
        DownStairParam param;

        if (motion_selector_)
            motion_selector_(params, param);

        for (auto i : params)
        {
            if (i.first == "step_height")
            {
                param.step_height = std::stod(i.second);
            }
            else if (i.first == "turning_rate")
            {
                param.turning_rate = std::stod(i.second);
            }
            else if (i.first == "period")
            {
                param.period = std::stod(i.second);
            }
        }
        msg_out.copyStruct(param);
        return true;
    }

    // supporting functions for walk gait
    void DownStairPlanner::calculate_coe(double* t, double* x, double* v, double* coe)
    {
        double t1 = t[0];
        double t2 = t[1];
        double x1 = x[0];
        double x2 = x[1];
        double v1 = v[0];
        double v2 = v[1];
        double dt = t1 - t2;
        coe[0] = ((t1 - t2) * (v1 + v2) - 2 * x1 + 2 * x2)/dt/dt/dt;
        coe[1] = (-t1*t1 * (v1 + 2 * v2) + t1 * (-t2 * v1 + t2 * v2 + 3 * x1 - 3 * x2) +  t2 * (2 * t2 * v1 + t2 * v2 + 3 * x1 - 3 * x2))/dt/dt/dt;
        coe[2] = (1/dt/dt/dt) * (-t2*t2*t2 * v1 + t1*t1*t1 * v2 + t1*t1 * t2 * (2 * v1 + v2) -   t1 * t2 * (t2 * v1 + 2 * t2 * v2 + 6 * x1 - 6 * x2));
        coe[3] = (1/dt/dt/dt) * (t2 * (t1 * (-t1 + t2) * (t2 * v1 + t1 * v2) - t2 * (-3 * t1 + t2) * x1) + t1*t1 * (t1 - 3 * t2) * x2);
    }

    void DownStairPlanner::waistForwardKinematic(double &theta2, double &theta1,double &d)
    {
        //theta2=groupA angle ;theta1=groupB angle
        double AB=250;
        double AC=183;
        double theta0=45*kinematics::PI/180.0;
        //int nTuringTime=ceil(abs(theta0))
        double BC0=176.8862;
        double theta=theta2-theta1+theta0;
        double BC=sqrt(AB*AB+AC*AC-2*AB*AC*cos(theta));
        d=BC-BC0;
        //double k=16000/3;
        //double motor13Angle=d*k;
       // return motor13Angle;
    }

    /* XML configuration

            <sc default="sc_param">
              <sc_param type="group" >
                <motor_select type="unique" default="all">
                  <all abbreviation="a"/>
                  <first abbreviation="f"/>
                  <second abbreviation="s"/>
                  <motor abbreviation="m" type="intArray" default="0"/>
                  <physical_motor abbreviation="p" type="intArray" default="0"/>
                  <leg abbreviation="l" type="intArray" default="0"/>
                </motor_select>
                <step_length  abbreviation="d" type="double" default="0.15"/>
                <step_height  abbreviation="h" type="double" default="0.05"/>
                <step_number  abbreviation="n" type="int"    default="1"/>
                <turning_rate abbreviation="b" type="double" default="0.0"/>
                <period       abbreviation="t" type="double" default="2"/>
              </sc_param>
            </sc>
    */
}
