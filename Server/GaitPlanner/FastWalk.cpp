#include "FastWalk.h"

void dlmread(const char *FileName, double *pMatrix)
{
    std::ifstream file;

    file.open(FileName);

    if (!file) throw std::logic_error("file not exist");

    int i = 0;
    while (!file.eof())
    {
        file >> *(pMatrix + i);
        ++i;
    }
}

namespace robot_app
{
    aris::server::MotionSelector FastWalk::motion_selector_ = nullptr;
    int FastWalk::total_count_;
    double FastWalk::begin_foot_pos_[18];
    double FastWalk::begin_body_pos_[3];
    double FastWalk::foot_pos_[18];
    double FastWalk::body_pos_[3];
    double FastWalk::begin_joint_position_[kinematics::MOTION_NUM];
    double FastWalk::foot_cmd_pos_[kinematics::MOTION_NUM];
    double FastWalk::joint_cmd_pos_[kinematics::MOTION_NUM];
    double FastWalk::pos2count_ratio_[kinematics::MOTION_NUM];
    double FastWalk::cubic_coefs1_[4];
    double FastWalk::cubic_coefs2_[4];

    double FastWalk::swingPee_scaling[2000][2];
    const double FastWalk::initTipPos[2] {0,-0.45};

    void FastWalk::setMotionSelector(const aris::server::MotionSelector &selector)
    {
        motion_selector_ = selector;
    }

    int FastWalk::fastWalk(aris::model::Model &model, aris::server::PlanParamBase &param)
    {
        using aris::control::EthercatMotion;
        using kinematics::PI;

        auto& fw_param = static_cast<WalkParam &>(param);
        static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

        /////////////////////////////////////////////////////////////
        // Initialize beginning states here                        //
        /////////////////////////////////////////////////////////////
        if (fw_param.count == 0)
        {
            // get pos2count ratio and begin joint position for each motor
            for (int i = 0; i < kinematics::MOTION_NUM; i++)
            {
                pos2count_ratio_[i] = cs.controller().motionAtAbs(i).pos2countRatio();
                begin_joint_position_[i] = (*fw_param.motion_raw_data)[i].feedback_pos / pos2count_ratio_[i];
            }

            // calculate cubic coefficients for body trj interpolation
            total_count_ = (int)(fw_param.period * cs.getControlFreq());
            double t[2]={0, fw_param.period};
            double x[2]={0, fw_param.step_length / 4};
            double v1[2]={0, 0.5 * fw_param.step_length / fw_param.period};
            double v2[2]={v1[1], 0};
            calculate_coe(t, x, v1, cubic_coefs1_);
            calculate_coe(t, x, v2, cubic_coefs2_);

            // set body's begin pos
            std::fill(begin_body_pos_, begin_body_pos_+3, 0);

            // set leg's begin pos
            for (int i = 0; i < kinematics::ACTIVE_LEG_NUM; i++)
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

        int period_count = fw_param.count % total_count_;
        double const_v = 0.5 * fw_param.step_length / fw_param.period;
        double instant_v {0};
        double t_r = (period_count + 1) * 1.0 / cs.getControlFreq();
        double t_r2 = t_r * t_r;
        double t_r3 = t_r2 * t_r;

        // trajectory planning
        if (fw_param.count / total_count_ == 0)
        {
            // the first step
            //swing
            instant_v = const_v / fw_param.period * period_count / cs.getControlFreq();
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_A[i];
                foot_cmd_pos_[2*legID]=initTipPos[0] + (swingPee_scaling[period_count][0]-initTipPos[0]) * instant_v / const_v;
                foot_cmd_pos_[2*legID+1]=swingPee_scaling[period_count][1];
            }

            //stance
            body_pos_[0] = begin_body_pos_[0] + cubic_coefs1_[0] * t_r3 + cubic_coefs1_[1] * t_r2 +
                                   cubic_coefs1_[2] * t_r + cubic_coefs1_[3];
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_B[i];
                foot_cmd_pos_[2*legID]=begin_foot_pos_[3*legID] - body_pos_[0];
                foot_cmd_pos_[2*legID+1]=begin_foot_pos_[3*legID+1];
            }
        }
        else if (fw_param.count/total_count_ == 2 * fw_param.step_number - 1)
        {
            //the last step
            //swing
            instant_v = const_v - const_v / fw_param.period * period_count / cs.getControlFreq();
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_B[i];
                foot_cmd_pos_[2*legID]=initTipPos[0] + (swingPee_scaling[period_count][0]-initTipPos[0]) * instant_v / const_v;
                foot_cmd_pos_[2*legID+1]=swingPee_scaling[period_count][1];
            }

            //stance
            body_pos_[0] = begin_body_pos_[0] + cubic_coefs2_[0] * t_r3 + cubic_coefs2_[1] * t_r2 +
                                   cubic_coefs2_[2] * t_r + cubic_coefs2_[3];
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_A[i];
                foot_cmd_pos_[2*legID]=begin_foot_pos_[3*legID] + fw_param.step_length/4 - body_pos_[0];
                foot_cmd_pos_[2*legID+1]=begin_foot_pos_[3*legID+1];
            }
        }
        else if (fw_param.count/total_count_ % 2 == 1) //constant velocity stage
        {
            //swing
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_B[i];
                foot_cmd_pos_[2*legID]=swingPee_scaling[period_count][0];
                foot_cmd_pos_[2*legID+1]=swingPee_scaling[period_count][1];
            }

            //stance
            body_pos_[0] = begin_body_pos_[0] + (fw_param.step_length/2)*((period_count+1)*1.0/total_count_);
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_A[i];
                foot_cmd_pos_[2*legID]=begin_foot_pos_[3*legID] + fw_param.step_length/4 - body_pos_[0];
                foot_cmd_pos_[2*legID+1]=begin_foot_pos_[3*legID+1];
            }
        }
        else
        {
            //swing
            instant_v = const_v / fw_param.period * fw_param.count / cs.getControlFreq();
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_A[i];
                foot_cmd_pos_[2*legID]=swingPee_scaling[period_count][0];
                foot_cmd_pos_[2*legID+1]=swingPee_scaling[period_count][1];
            }

            //stance
            body_pos_[0] = begin_body_pos_[0] + (fw_param.step_length/2)*((period_count+1)*1.0/total_count_);
            for (int i = 0; i < 3; i++)
            {
                int legID = kinematics::LEG_GROUP_B[i];
                foot_cmd_pos_[2*legID]=begin_foot_pos_[3*legID] + fw_param.step_length/4 - body_pos_[0];
                foot_cmd_pos_[2*legID+1]=begin_foot_pos_[3*legID+1];
            }
        }

        rt_printf("foot_cmd_pos_:\n");
        for(int i=0;i<2;i++)
        {
            rt_printf("%.2f,%.2f\n",foot_cmd_pos_[2*i],foot_cmd_pos_[2*i+1]);
        }
        
        // Leg joint cmd
        for (int i = 0; i < kinematics::ACTIVE_LEG_NUM; i++)
        {
            // Calculate joint position
            kinematics::Leg::LegIK(
                &(foot_cmd_pos_[i*2]),
                &(joint_cmd_pos_[i*2]),
                kinematics::LEG_ORIENTATION[i]);
        }
        // Waist joint cmd
        joint_cmd_pos_[kinematics::WAIST_INDEX] = begin_joint_position_[kinematics::WAIST_INDEX];

        ///////////////////////////////////////////////////////
        // User Planning code ends here                      //
        ///////////////////////////////////////////////////////

        // don't change the following part unless you DO know what you are doing
        for (int i = 0; i < kinematics::MOTION_NUM; i++)
        {
            if (fw_param.active_motor[i])
            {
                (*fw_param.motion_raw_data)[i].cmd = EthercatMotion::Cmd::RUN;
                (*fw_param.motion_raw_data)[i].mode = EthercatMotion::Mode::POSITION;
                (*fw_param.motion_raw_data)[i].target_pos = joint_cmd_pos_[i] * pos2count_ratio_[i];
            }
        }

        // return 0 if planning finished, otherwise return positive value
        return 2 * fw_param.step_number * total_count_ - fw_param.count -1;
    }

    bool FastWalk::fastWalkParser(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)
    {
        WalkParam param;

        if (motion_selector_)
            motion_selector_(params, param);

        for (auto i : params)
        {
            if (i.first == "step_length")
            {
                param.step_length = std::stod(i.second);
            }
            else if (i.first == "step_height")
            {
                param.step_height = std::stod(i.second);
            }
            else if (i.first == "step_number")
            {
                param.step_number = std::stoi(i.second);
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

        GetScalingPath(param);

        msg_out.copyStruct(param);
        return true;
    }

    void FastWalk::GetScalingPath(WalkParam &param)
    {
        double aLmt {1e6/4096*2 * kinematics::PI / 81 * 1.0};
        double vLmt {100 * kinematics::PI / 81 * 1.0};
        double out_TipPos[4000][2] {0};
        double out_period {0};
        int out_period_count {0};

        time_optimal::TimeOptimalMotionSingleEffector planner;
        planner.GetTimeOptimalGait(param.step_length,param.step_height,aLmt,vLmt,initTipPos[1],*out_TipPos,out_period);
        printf("out_period is %.3f\n",out_period);

        out_period_count=(int)(out_period*1000);

        int totalCount = (int)(param.period * 1000);
        int k {0};
        double ratio = (double)totalCount/out_period_count;//>=1
        for(int i=0; i<totalCount-1; i++)
        {
            for(int j=k; j<out_period_count; j++)
            {
                if(i >= ratio*j && i < ratio*(j+1))
                {
                    swingPee_scaling[i][0]=out_TipPos[j][0]+(out_TipPos[j+1][0]-out_TipPos[j][0])*(i-ratio*j)/ratio;
                    swingPee_scaling[i][1]=out_TipPos[j][1]+(out_TipPos[j+1][1]-out_TipPos[j][1])*(i-ratio*j)/ratio;
                    k=j;
                    break;
                }
            }
        }
        swingPee_scaling[totalCount-1][0]=out_TipPos[out_period_count-1][0];
        swingPee_scaling[totalCount-1][1]=out_TipPos[out_period_count-1][1];
    }

    // supporting functions for walk gait
    void FastWalk::calculate_coe(double* t, double* x, double* v, double* coe)
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

    /* XML configuration
            <fwk default="fwk_param">
              <fwk_param type="group" >
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
                <step_number abbreviation="n" type="int" default="2"/>
                <turning_rate abbreviation="b" type="double" default="0.0"/>
                <period abbreviation="t" type="double" default="2"/>
              </fwk_param>
            </fwk>
    */
}
