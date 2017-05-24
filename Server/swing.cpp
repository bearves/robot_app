#include "swing.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto swingParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    swParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "xAngle")
        {
            param.xAngle = stod(i.second) / 180 * PI;
        }
        else if (i.first == "yAngle")
        {
            param.yAngle = stod(i.second) / 180 * PI;
        }
        else if (i.first == "zAngle")
        {
            param.zAngle = stod(i.second) / 180 * PI;
        }
        else if (i.first == "rDistance")
        {
            param.rDistance = stod(i.second);
        }
        else if (i.first == "yDistance")
        {
            param.yDistance = stod(i.second);
        }
    }

    msg.copyStruct(param);
}

auto swingGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const swParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    const double s = -PI * cos(PI * (param.count + 1) / param.totalCount) + PI;//s 从0到2*PI.

    if (0 != param.xAngle)//秋千平面法线沿x方向
    {
        Peb[0] = 0;
        Peb[1] = param.rDistance * (1 - cos(param.xAngle * sin(s)));
        Peb[2] = param.rDistance * sin(param.xAngle * sin(s));
        Peb[3] = 0;
        Peb[4] = -param.xAngle * sin(s);
        Peb[5] = 0;
        robot.SetPeb(Peb, beginMak);
        robot.SetPee(Pee, beginMak);
    }
    else if (0 != param.zAngle)//秋千平面法线沿z方向
    {
        Peb[0] = param.rDistance * sin(param.zAngle * sin(s));
        Peb[1] = param.rDistance * (1 - cos(param.zAngle * sin(s)));
        Peb[2] = 0;
        Peb[3] = param.zAngle * sin(s);
        Peb[4] = 0;
        Peb[5] = 0;
        robot.SetPeb(Peb, beginMak);
        robot.SetPee(Pee, beginMak);
    }
    else if (0 != param.yAngle)//螺旋运动沿y方向
    {
        Peb[0] = 0;
        Peb[1] = param.yDistance * (1 - std::cos(2 * s)) / 2;
        Peb[2] = 0;
        Peb[3] = 0;
        Peb[4] = param.yAngle * std::sin(s);
        Peb[5] = 0;
        robot.SetPeb(Peb, beginMak, "123");
        robot.SetPee(Pee, beginMak);
    }
    else
    {
        robot.SetPeb(Peb, beginMak);
        robot.SetPee(Pee, beginMak);
    }
    return param.totalCount - param.count - 1;
}
