#include "find_joint_center.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto findJointCenterParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    fjcParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "leg")
        {
            param.leg_id = std::stoi(i.second);
			if (param.leg_id < 0 || param.leg_id > 5)
				throw std::runtime_error("invalid leg_id");
			std::fill_n(param.active_motor, 18, false);
			std::fill_n(param.active_motor + param.leg_id * 3, 3, true);
       }
        else if (i.first == "chain")
        {
            param.chain_id = std::stoi(i.second);
        }
        else if (i.first == "distance")
        {
            param.distance = std::stod(i.second);
        }
    }

    msg.copyStruct(param);
}

auto findJointCenterGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const fjcParam &>(param_in);

    //初始化
    static double beginPin[18];
    //static double beginLegPin[3];

    if (param.count == 0)
    {
        std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
        //std::copy_n(beginPin + 3 * param.leg_id, 3, beginLegPin);
    }

    int period_count = param.count%param.totalCount;
    int sign = param.count < param.totalCount ? 1 : -1;
    const double s = -PI*cos(PI * (period_count + 1) / param.totalCount) + PI; //s从0到2*PI. 

    double Pin[3];
    std::copy_n(beginPin + 3 * param.leg_id, 3, Pin);
    int i1 = (param.chain_id + 1) % 3;
    int i2 = (param.chain_id + 2) % 3;
    Pin[i1] += param.distance * std::sin(s);
    Pin[i2] += sign * param.distance * std::sin(s);

    //robot.pLegs[param.leg_id]->SetPin(Pin);
    for (int i = 0; i < 3; ++i)
    {
        robot.motionPool().at(param.leg_id * 3 + i).setMotPos(Pin[i]);
    }

    return 2*param.totalCount - param.count - 1;
}
