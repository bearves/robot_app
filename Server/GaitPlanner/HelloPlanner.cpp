#include "HelloPlanner.h"

namespace robot_app
{
    aris::server::MotionSelector HelloPlanner::motion_selector_ = nullptr;
    int HelloPlanner::total_count_;
    double HelloPlanner::begin_foot_pos_[18];
    double HelloPlanner::begin_body_pos_[3];
    double HelloPlanner::foot_pos_[18];
    double HelloPlanner::body_pos_[3];
    double HelloPlanner::begin_joint_position_[kinematics::MOTION_NUM];
    double HelloPlanner::foot_cmd_pos_[kinematics::MOTION_NUM];
    double HelloPlanner::joint_cmd_pos_[kinematics::MOTION_NUM];
    double HelloPlanner::pos2count_ratio_[kinematics::MOTION_NUM];
    double HelloPlanner::body[6];
    double HelloPlanner::begin_2_pos[12];
    int HelloPlanner::total_count_part;
    double HelloPlanner::tip_[18]=  {   0.445  ,  0.0,        0,//F
               0.2525,   0.0, -0.25902,//FL
             -0.2525,   0.0, -0.25902,//RL
              -0.445  ,  0.0,        0,//R
             -0.2525,   0.0,  0.25902,//RR
              0.2525,   0.0,  0.25902 };//FR

    void HelloPlanner::setMotionSelector(const aris::server::MotionSelector &selector)
    {
        motion_selector_ = selector;
    }

    int HelloPlanner::hello(aris::model::Model &model, aris::server::PlanParamBase &param)
    {
        using aris::control::EthercatMotion;
        using kinematics::PI;

        auto& hello_param = static_cast<HelloParam &>(param);//YK param define count to reflect actual count time
        static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

        /////////////////////////////////////////////////////////////
        // Initialize beginning states here                        //
        /////////////////////////////////////////////////////////////
        /*if (hello_param.count==1)
        {
            std::cout<<"the first motor pos = "<<joint_cmd_pos_[0]<<std::endl;
        }*/
        
        if (hello_param.count == 0)
        {
            // get pos2count ratio and begin joint position for each motor
            for (int i = 0; i < kinematics::MOTION_NUM; i++)
            {
                pos2count_ratio_[i] = cs.controller().motionAtAbs(i).pos2countRatio();//yk get ratio by .xml file
                begin_joint_position_[i] = (*hello_param.motion_raw_data)[i].feedback_pos / pos2count_ratio_[i];
            }

            // calculate cubic coefficients for body trj interpolation
            total_count_ = (int)(hello_param.period * cs.getControlFreq()); // YK every dait period
             total_count_part=int(0.2*total_count_);

            // set body's begin pos
            std::fill(begin_body_pos_, begin_body_pos_+3, 0);

            // set leg's begin pos
            for (int i = 0; i < kinematics::ACTIVE_LEG_NUM; i++)   //YK
            {
                kinematics::Leg::LegFK(
                    &(begin_joint_position_[i*2]),
                    &(begin_foot_pos_[i*3]),
                    kinematics::LEG_ORIENTATION[i]);
                begin_foot_pos_[i*3+2] = 0;

                rt_printf("Leg %d's begin position is (%.3f, %.3f)\n",
                    i,
                    begin_foot_pos_[i*3],
                    begin_foot_pos_[i*3+1]);
            }
            std::copy(begin_foot_pos_, begin_foot_pos_+18, foot_pos_);
            std::copy(begin_body_pos_, begin_body_pos_+3,  body_pos_);
        }

        ///////////////////////////////////////////////////////////////////
        // User Planning code begins here                                //
        // You need to calculate joint_cmd_pos_ in this part             //
        ///////////////////////////////////////////////////////////////////
        int period_count = hello_param.count % total_count_part;
        if (hello_param.count>=0 && hello_param.count<total_count_part)
        {
            //the first phase;
            double y1=hello_param.height_hello;
            double y0=kinematics::STANDING_HEIGHT;
            double beta0=0;
            double y=0.5*(y0+y1)-0.5*(y1-y0)*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            double beta=0.5*(beta0+hello_param.beta1)-0.5*(hello_param.beta1-beta0)*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            body[0]=0;
            body[1]=y;
            body[2]=0;
            body[3]=beta;
            body[4]=0;
            body[5]=0;
            kinematics::O13Inverse::InverseSolve(
                &(tip_[0]),
                &(body[0]),
                &(joint_cmd_pos_[0]));
        /*if (hello_param.count%200 ==0)
        {
            std::cout<<"count="<<hello_param.count<<"the first motor pos = "<<joint_cmd_pos_[0]<<"beta="<<beta<<"period_count="<<period_count<<"beta1="<<hello_param.beta1<<"total_count_part="<<total_count_part<<std::endl;
            //std::cout<<"hello_param.alpha1="<<hello_param.alpha1<<" hello_param.height_hello="<<hello_param.height_hello<<"hello_param.beta1="<<hello_param.beta1<<"hello_param.period="<<hello_param.period<<"hello_param.x2"<<hello_param.x2<<"hello_param.y2"<<hello_param.y2<<std::endl;
        }*/
        /*if (hello_param.count ==1999)
        {
            std::cout<<"count="<<hello_param.count<<"the first motor pos = "<<joint_cmd_pos_[0]<<std::endl;
            for (int i=0; i<18;i++)
            {
                std::cout<<i<<"tip_[i]"<<tip_[i]<<std::endl;
            }
            for (int i=0; i<6;i++)
            {
                std::cout<<i<<"body[i]"<<body[i]<<std::endl;
            }
        }*/

        }
        if ((hello_param.count>=total_count_part && hello_param.count<2*total_count_part))
        {
            //the second phase
            double y1=hello_param.height_hello;
            body[0]=0;
            body[1]=y1;
            body[2]=0;
            body[3]=hello_param.beta1;
            body[4]=0;
            body[5]=0;
            if (hello_param.count ==2000)
            {
                  for (int i=0; i<18;i++)
                 {
                         std::cout<<i<<"before_tip_[i]"<<tip_[i]<<std::endl;
                 }
            }

            kinematics::O13InverseLeg::InverseSolveLeg13(
                &(tip_[0]),
                &(body[0]),
                &(begin_2_pos[0]));
            double x=0.5*(begin_2_pos[0]+hello_param.x2)-0.5*(hello_param.x2-begin_2_pos[0])*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            double y=0.5*(begin_2_pos[1]+hello_param.y2)-0.5*(hello_param.y2-begin_2_pos[1])*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            foot_cmd_pos_[0]=x;
            foot_cmd_pos_[1]=y;
            foot_cmd_pos_[4]=x;
            foot_cmd_pos_[5]=y;
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[0]),
                &(joint_cmd_pos_[0]),
                1);
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[4]),
                &(joint_cmd_pos_[4]),
                1);

        /*if (hello_param.count ==2000)
        {
            std::cout<<"count="<<hello_param.count<<" the first motor pos = "<<joint_cmd_pos_[0]<<" begin_2_pos[0]="<<begin_2_pos[0]<<" begin_2_pos[1]="<<begin_2_pos[1]<<std::endl;
            for (int i=0; i<18;i++)
            {
                std::cout<<i<<"back_tip_[i]"<<tip_[i]<<std::endl;
            }
            for (int i=0; i<6;i++)
            {
                std::cout<<i<<"body[i]"<<body[i]<<std::endl;
            }

        }*/
        }
        if (hello_param.count>=2*total_count_part && hello_param.count<3*total_count_part)
        {
            //the third phase
            double r=std::sqrt(hello_param.x2*hello_param.x2+hello_param.y2*hello_param.y2);
            double sx=-PI*std::cos( PI * (period_count + 1) *1.0 / total_count_part)+PI;
            double beta0=atan2(hello_param.y2,hello_param.x2);
            double x1=r*cos(beta0+hello_param.alpha1*sin(sx));
            double y1=r*sin(beta0+hello_param.alpha1*sin(sx));
            double x2=r*cos(beta0-hello_param.alpha1*sin(sx));
            double y2=r*sin(beta0-hello_param.alpha1*sin(sx));
            foot_cmd_pos_[0]=x1;
            foot_cmd_pos_[1]=y1;
            foot_cmd_pos_[4]=x2;
            foot_cmd_pos_[5]=y2;
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[0]),
                &(joint_cmd_pos_[0]),
                1);
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[4]),
                &(joint_cmd_pos_[4]),
                1);
        }
        if (hello_param.count>=3*total_count_part && hello_param.count<4*total_count_part)
        {
            // the 4th phase
            double y1=hello_param.height_hello;
            body[0]=0;
            body[1]=y1;
            body[2]=0;
            body[3]=hello_param.beta1;
            body[4]=0;
            body[5]=0;

            kinematics::O13InverseLeg::InverseSolveLeg13(
                &(tip_[0]),
                &(body[0]),
                &(begin_2_pos[0]));
            double x=0.5*(begin_2_pos[0]+hello_param.x2)-0.5*(-hello_param.x2+begin_2_pos[0])*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            double y=0.5*(begin_2_pos[1]+hello_param.y2)-0.5*(-hello_param.y2+begin_2_pos[1])*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            foot_cmd_pos_[0]=x;
            foot_cmd_pos_[1]=y;
            foot_cmd_pos_[4]=x;
            foot_cmd_pos_[5]=y;
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[0]),
                &(joint_cmd_pos_[0]),
                1);
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[4]),
                &(joint_cmd_pos_[4]),
                1);
        }
        if (hello_param.count>=4*total_count_part && hello_param.count<5*total_count_part)
        {
            // the 5th second
            double y1=hello_param.height_hello;
            double y0=kinematics::STANDING_HEIGHT;
            double beta0=0;
            double y=0.5*(y0+y1)-0.5*(-y1+y0)*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            double beta=0.5*(beta0+hello_param.beta1)-0.5*(-hello_param.beta1+beta0)*std::cos( PI * (period_count + 1) *1.0 / total_count_part);
            body[0]=0; body[1]=y; body[2]=0; body[3]=beta; body[4]=0; body[5]=0;
            kinematics::O13Inverse::InverseSolve(
                &(tip_[0]),
                &(body[0]),
                &(joint_cmd_pos_[0]));
        }

        ///////////////////////////////////////////////////////
        // User Planning code ends here                      //
        ///////////////////////////////////////////////////////

        // don't change the following part unless you DO know what you are doing
        for (int i = 0; i < kinematics::MOTION_NUM; i++)
        {
            if (hello_param.active_motor[i])
            {
                (*hello_param.motion_raw_data)[i].cmd = EthercatMotion::Cmd::RUN;
                (*hello_param.motion_raw_data)[i].mode = EthercatMotion::Mode::POSITION;
                (*hello_param.motion_raw_data)[i].target_pos = joint_cmd_pos_[i] * pos2count_ratio_[i];
            }
        }

        // return 0 if planning finished, otherwise return positive value
        return  total_count_ - hello_param.count -1;

    }
    bool HelloPlanner::helloParser(const std::string &cmd,const std::map<std::string, std::string> &params,aris::core::Msg &msg_out)
    {
        HelloParam param;
        if (motion_selector_)
            motion_selector_(params, param);

        for (auto i : params)
        {
            if (i.first == "alpha1")
            {
                param.alpha1 = std::stod(i.second);
            }
            else if (i.first == "height_hello")
            {
                param.height_hello = std::stod(i.second);
            }
            else if (i.first == "beta1")
            {
                param.beta1 = std::stod(i.second);
            }
            else if (i.first == "period")
            {
                param.period = std::stod(i.second);
            }
            else if (i.first == "x2")
            {
                param.x2 = std::stod(i.second);
            }
            else if (i.first == "y2")
            {
                param.y2 = std::stod(i.second);
            }
        }
        msg_out.copyStruct(param);
        return true;
    }
}

