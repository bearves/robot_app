#include "calibration.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto calibrationParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    caliParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "leg")
        {
            param.leg_id = stoi(i.second);
			if (param.leg_id < 0 || param.leg_id > 5)
				throw std::runtime_error("invalid leg_id");
			std::fill_n(param.active_motor, 18, false);
			std::fill_n(param.active_motor + param.leg_id * 3, 3, true);
        }
        else if (i.first == "next")
        {
            CaliCmd::getCmd().cmd() = Command::NEXT;
        }
        else if (i.first == "previous")
        {
            CaliCmd::getCmd().cmd() = Command::PREVIOUS;
        }
        else if (i.first == "quit")
        {
            CaliCmd::getCmd().cmd() = Command::QUIT;
        }
    }
    msg.copyStruct(param);
}

auto calibrationGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const caliParam &>(param_in);

    const double targetPee[][3]={ 
//        { 0.600, -0.100, -0.100 },
//        { 0.600,  0.000, -0.100 },
//        { 0.600,  0.100, -0.100 },
        { 0.650, -0.100, -0.100 },
        { 0.650,  0.000, -0.100 },
        { 0.650,  0.100, -0.100 },
        { 0.700, -0.100, -0.100 },
        { 0.700,  0.000, -0.100 },
        { 0.700,  0.100, -0.100 },
//        { 0.600, -0.100,  0.000 },
//        { 0.600,  0.000,  0.000 },
//        { 0.600,  0.100,  0.000 },
        { 0.650, -0.100,  0.000 },
        { 0.650,  0.000,  0.000 },
        { 0.650,  0.100,  0.000 },
        { 0.700, -0.100,  0.000 },
        { 0.700,  0.000,  0.000 },
        { 0.700,  0.100,  0.000 },
//        { 0.600, -0.100,  0.100 },
//        { 0.600,  0.000,  0.100 },
//        { 0.600,  0.100,  0.100 },
        { 0.650, -0.100,  0.100 },
        { 0.650,  0.000,  0.100 },
        { 0.650,  0.100,  0.100 },
        { 0.700, -0.100,  0.100 },
        { 0.700,  0.000,  0.100 },
        { 0.700,  0.100,  0.100 }};

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double lastPee[3];
    static bool isRunning;
    static int target_id;
    static std::int32_t begin_count;

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        robot.pLegs[param.leg_id]->GetPee(lastPee, robot.pLegs[param.leg_id]->base());
        isRunning = false;
        target_id = -1;
        begin_count = 0;

        rt_printf("Initial pee of leg %d is: %f %f %f\n", param.leg_id,
            lastPee[0], lastPee[1], lastPee[2]);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    if (isRunning)
    {
        std::int32_t period_count = param.count - begin_count;
        const double s = -PI/2 * cos(PI * (period_count + 1) / param.totalCount) + PI/2;//s 从0到PI.
        double legPee[3];

        for (int i = 0; i < 3; ++i)
        {
            legPee[i] = lastPee[i] * (1 + cos(s)) / 2 + targetPee[target_id][i] * (1 - cos(s)) / 2;
        }

        robot.pLegs[param.leg_id]->SetPee(legPee, robot.pLegs[param.leg_id]->base());

        if (period_count + 1 == param.totalCount)
        {
            isRunning = false;
            CaliCmd::getCmd().cmd() = Command::NONE;
            std::copy_n(targetPee[target_id], 3, lastPee);
            rt_printf("Calibration Point No. %d\n", target_id);
            double testPee[3];
            robot.pLegs[param.leg_id]->GetPee(testPee, robot.pLegs[param.leg_id]->base());
            rt_printf("testPee: %f %f %f\n", testPee[0], testPee[1], testPee[2]);
        }
    }
    else
    {
        switch (CaliCmd::getCmd().cmd())
        {
        case Command::NEXT:
            ++target_id;
            if (target_id >= 18)
            {
                return 0;
            }
            begin_count = param.count + 1;
            isRunning = true;
            break;
        case Command::PREVIOUS:
            --target_id;
            begin_count = param.count + 1;
            isRunning = true;
            break;
        case Command::QUIT:
            return 0;
            break;
        default:
            break;
        }
    }

    return 1;
}
