#include "say_hello.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto sayHelloParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    shParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "stepLength")
        {
            param.stepLength = std::stod(i.second);
        }
        else if (i.first == "stepHeight")
        {
            param.stepHeight = std::stod(i.second);
        }
        else if (i.first == "bodyUp")
        {
            param.bodyUp = std::stod(i.second);
        }
        else if (i.first == "bodyPitch")
        {
            param.bodyPitch = std::stod(i.second) / 180 * PI;
        }
        else if (i.first == "helloAmplitude")
        {
            param.helloAmplitude = std::stod(i.second);
        }
        else if (i.first == "helloTimes")
        {
            param.helloTimes = std::stoi(i.second);
        }
        else if (i.first == "forward")
        {
            param.isForward = true;
        }
        else if (i.first == "backward")
        {
            param.isForward = false;
        }
    }

    msg.copyStruct(param);
}

auto sayHelloGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const shParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    int totalCount = param.totalCount;
    int n = param.helloTimes;
    const double sign = param.isForward ? 1 : -1;
    double footDist = std::sqrt(std::pow(beginPee[2], 2) + std::pow(beginPee[1] - param.bodyUp, 2));
    double theta = std::atan2(std::fabs(beginPee[2]), std::fabs(beginPee[1]) + param.bodyUp);

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);

    double targetPeb[6];
    std::fill(targetPeb, targetPeb + 6, 0);
    targetPeb[1] = param.bodyUp;
    targetPeb[4] = sign * param.bodyPitch;

    double state1Pee[18]; //调整中间腿
    std::copy(beginPee, beginPee + 18, state1Pee);
    state1Pee[5] -= sign * param.stepLength;
    state1Pee[14] -= sign * param.stepLength;

    double state2Pee[18]; //举起前（或后）腿
    std::copy(state1Pee, state1Pee + 18, state2Pee);
    for (int i = 0; i < 6; i += 3)
    {
        int leg_id = param.isForward ? i : (5 - i);
        state2Pee[3 * leg_id + 1] = param.bodyUp - footDist * std::cos(sign * (2 * param.bodyPitch + theta));
        state2Pee[3 * leg_id + 2] = -footDist * std::sin(sign * (2 * param.bodyPitch + theta));
    }

    //第一步：迈中间腿
    if (param.count < param.totalCount)
    {
        const double s = -PI / 2 * std::cos(PI * (param.count + 1) / param.totalCount) + PI / 2; //s从0到PI.
        std::copy(beginPee, beginPee + 18, Pee);
        for (int i = 1; i < 6; i += 3)
        {
            int leg_id = param.isForward ? i : (5 - i);
            Pee[3 * leg_id + 1] += param.stepHeight * std::sin(s);
            Pee[3 * leg_id + 2] -= sign * param.stepLength * (1 - std::cos(s)) / 2;
        }
    }
    //第二步：抬头，抬前腿
    else if (param.count < 2 * param.totalCount)
    {
        const double s = -0.5 * std::cos(PI * (param.count + 1 - param.totalCount) / param.totalCount) + 0.5; //s从0到1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * s;
        }
        std::copy(state1Pee, state1Pee + 18, Pee);
        for (int i = 0; i < 6; i += 3)
        {
            int leg_id = param.isForward ? i : (5 - i);
            Pee[3 * leg_id + 1] = state1Pee[3 * leg_id + 1] * (1 - s) + state2Pee[3 * leg_id + 1] * s;
            Pee[3 * leg_id + 2] = state1Pee[3 * leg_id + 2] * (1 - s) + state2Pee[3 * leg_id + 2] * s;
        }
    }
    //第三步：晃前腿打招呼
    else if (param.count < (2 + n) * param.totalCount)
    {
        const double s = -PI * std::cos(PI * (param.count % param.totalCount + 1) / param.totalCount) + PI; //s从0到2*n*PI.
        std::copy(targetPeb, targetPeb + 6, Peb);
        std::copy(state2Pee, state2Pee + 18, Pee);
        for (int i = 0; i < 6; i += 3)
        {
            int leg_id = param.isForward ? i : (5 - i);
            Pee[3 * leg_id + 1] = state2Pee[3 * leg_id + 1] - std::pow(-1, leg_id) * param.helloAmplitude * std::sin(s) * std::cos(theta);
            Pee[3 * leg_id + 2] = state2Pee[3 * leg_id + 2] + sign * std::pow(-1, leg_id) * param.helloAmplitude * std::sin(s) * std::sin(theta);
        }
    }
    //第四步：收前腿，恢复身体姿态
    else if (param.count < (3 + n) * param.totalCount)
    {
        const double s = -0.5 * std::cos(PI * (param.count + 1 - (2 + n) * param.totalCount) / param.totalCount) + 0.5; //s从0到1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * (1 - s);
        }
        std::copy(state2Pee, state2Pee + 18, Pee);
        for (int i = 0; i < 6; i += 3)
        {
            int leg_id = param.isForward ? i : (5 - i);
            Pee[3 * leg_id + 1] = state2Pee[3 * leg_id + 1] * (1 - s) + state1Pee[3 * leg_id + 1] * s;
            Pee[3 * leg_id + 2] = state2Pee[3 * leg_id + 2] * (1 - s) + state1Pee[3 * leg_id + 2] * s;
        }
    }
    //第五步：收中间腿
    else
    {
        const double s = -PI / 2 * std::cos(PI * (param.count + 1 - (3 + n) * param.totalCount) / param.totalCount) + PI / 2; //s从0到PI.
        std::copy(state1Pee, state1Pee + 18, Pee);
        for (int i = 1; i < 6; i += 3)
        {
            int leg_id = param.isForward ? i : (5 - i);
            Pee[3 * leg_id + 1] += param.stepHeight * std::sin(s);
            Pee[3 * leg_id + 2] += sign * param.stepLength * (1 - std::cos(s)) / 2;
        }
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return (4 + n) * param.totalCount - param.count - 1;
}
