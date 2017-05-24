#include "twist_waist.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto twistWaistParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    twParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "pitchMax")
        {
            param.pitchMax = std::stod(i.second) / 180 * PI;
        }
        else if (i.first == "rollMax")
        {
            param.rollMax = std::stod(i.second) / 180 * PI;
        }
        else if (i.first == "diameter")
        {
            param.diameter = std::stod(i.second);
        }
        else if (i.first == "height")
        {
            param.height = std::stod(i.second);
        }
    }

    msg.copyStruct(param);
}

auto twistWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const twParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    int totalCount = param.totalCount;
    int prepCount = param.totalCount / 6;
    int twistCount = param.totalCount * 4 / 6;

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    double targetPeb[6]{ 0 };
    std::fill(targetPeb, targetPeb + 6, 0);
    targetPeb[1] = param.height;
    targetPeb[2] = -param.diameter / 2;
    targetPeb[4] = param.pitchMax;

    if (param.count < prepCount)
    {
        const double s = -0.5 * cos(PI * (param.count + 1) / prepCount) + 0.5; //s从0到1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * s;
        }
        ////for test
        //if (param.count % 100 == 0)
        //{
        //	rt_printf("count:%d\n", param.count);
        //	rt_printf("s:%f\n", s);
        //	rt_printf("Peb:%f %f %f %f %f %f\n\n", Peb[0], Peb[1], Peb[2], Peb[3], Peb[4], Peb[5]);
        //}
    }
    else if (param.count < (prepCount + twistCount))
    {
        const double s = -PI * cos(PI * (param.count + 1 - prepCount) / twistCount) + PI; //s从0到2*PI.
        Peb[0] = param.diameter / 2 * sin(s);
        Peb[1] = targetPeb[1];
        Peb[2] = -param.diameter / 2 * cos(s);
        Peb[3] = param.rollMax * sin(s);
        Peb[4] = param.pitchMax * cos(s);
    }
    else
    {
        const double s = -0.5 * cos(PI * (param.count + 1 - prepCount - twistCount) / (totalCount - prepCount - twistCount)) + 0.5; //s从0到1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * (1 - s);
        }
        ////for test
        //if (param.count % 100 == 0)
        //{
        //	rt_printf("count:%d\n", param.count);
        //	rt_printf("%f\n", s);
        //	rt_printf("Peb:%f %f %f %f %f %f\n\n", Peb[0], Peb[1], Peb[2], Peb[3], Peb[4], Peb[5]);
        //}
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return param.totalCount - param.count - 1;
}
