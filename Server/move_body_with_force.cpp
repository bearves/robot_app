#include "move_body_with_force.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto moveBodyWithForceParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    mbfParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "h")
        {
            param.h = std::stod(i.second);
        }
        else if (i.first == "offset")
        {
            param.offset = std::stod(i.second);
        }
    }
    MbfState::getState().isStopping() = false;
    msg.copyStruct(param);
}

auto moveBodyWithForceStopParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    MbfState::getState().isStopping() = true;
}

auto moveBodyWithForceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const mbfParam &>(param_in);

    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    //行程检测
    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();
    double safetyOffset = param.offset;
    double pinUpBound[18]{ 0 };
    double pinLowBound[18]{ 0 };
    for (int i = 0; i < 18; ++i)
    {
        pinUpBound[i] = (double)cs.controller().motionAtAbs(i).maxPosCount() / cs.controller().motionAtAbs(i).pos2countRatio() - safetyOffset;
        pinLowBound[i] = (double)cs.controller().motionAtAbs(i).minPosCount() / cs.controller().motionAtAbs(i).pos2countRatio() + safetyOffset;
    }
    // for test
/*
    if (param.count == 0)
    {
        rt_printf("pinUpBound:\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n",
            pinUpBound[0], pinUpBound[1], pinUpBound[2], pinUpBound[3], pinUpBound[4], pinUpBound[5],
            pinUpBound[6], pinUpBound[7], pinUpBound[8], pinUpBound[9], pinUpBound[10], pinUpBound[11],
            pinUpBound[12], pinUpBound[13], pinUpBound[14], pinUpBound[15], pinUpBound[16], pinUpBound[17]);
        rt_printf("pinLowBound:\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n",
            pinLowBound[0], pinLowBound[1], pinLowBound[2], pinLowBound[3], pinLowBound[4], pinLowBound[5],
            pinLowBound[6], pinLowBound[7], pinLowBound[8], pinLowBound[9], pinLowBound[10], pinLowBound[11],
            pinLowBound[12], pinLowBound[13], pinLowBound[14], pinLowBound[15], pinLowBound[16], pinLowBound[17]);
    }
*/

    //行走控制
    static bool isWalking{ false };
    static int walkBeginCount{ 0 };
    static double walkBeginPee[18];

    //阻抗控制参数
    double F[6]{ 0, 0, 0, 0, 0, 0 };
    double M[6]{ 1, 1, 1, 1, 1, 1 };
    double K[6]{ 0, 0, 0, 0, 0, 0 };
    double C[6]{ 30, 30, 30, 20, 20, 20 };
    double bodyAcc[4]{ 0 };
    static double bodyVel[4]{ 0 };
    static double bodyDisp[4]{ 0 };
    const double kClockPeriod{ 0.001 };

    //力传感器数据
    static double forceOffsetSum[6]{ 0 };
    double forceOffsetAvg[6]{ 0 };
    double realForceData[6]{ 0 };
    double forceInBody[6]{ 0 };
    double forceThreshold[6]{ 10, 10, 10, 5, 5, 5 }; //力传感器的触发阈值,单位N或Nm
    double forceMax[6]{ 50, 50, 50, 20, 20, 20 }; //力的上限

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    //力传感器手动清零
    if (param.count < 100)
    {
        if (param.count == 0)
        {
            //静态变量重置
            std::fill(forceOffsetSum, forceOffsetSum + 6, 0);
            std::fill(bodyDisp, bodyDisp + 4, 0);
            std::fill(bodyVel, bodyVel + 4, 0);
            isWalking = false;
            walkBeginCount = 0;
        }
        for (int i = 0; i < 6; i++)
        {
            forceOffsetSum[i] += param.fzj_data->at(0).fsr_data[0].fce[i];
        }
    }
    else
    {
        //行走规划
        if (isWalking)
        {
            int walk_count = param.count - walkBeginCount;
            const int leg_begin_id = (walk_count / param.totalCount) % 2 == 1 ? 3 : 0;
            const double s = -(PI / 2)*cos(PI * (walk_count % param.totalCount + 1) / param.totalCount) + PI / 2;//s 从0到PI. 
            if (walk_count % param.totalCount == 0)
            {
                beginMak.setPrtPm(*robot.body().pm());
                beginMak.update();
                robot.GetPee(walkBeginPee, beginMak);
                //更新下一力控规划阶段的beginPee
                for (int i = 1; i < 18; i += 3)
                {
                    beginPee[i] = walkBeginPee[i];
                }
            }
            std::copy(walkBeginPee, walkBeginPee + 18, Pee);
            for (int i = leg_begin_id; i < 18; i += 6)
            {
                Pee[i] += (beginPee[i] - walkBeginPee[i])*(1 - std::cos(s)) / 2;
                Pee[i + 1] += param.h*std::sin(s);
                Pee[i + 2] += (beginPee[i + 2] - walkBeginPee[i + 2])*(1 - std::cos(s)) / 2;
            }

            robot.SetPeb(Peb, beginMak);
            robot.SetPee(Pee, beginMak);

            if (walk_count + 1 == 2 * param.totalCount)
            {
                std::fill(bodyDisp, bodyDisp + 4, 0);
                std::fill(bodyVel, bodyVel + 4, 0);
                isWalking = false;
            }
        }
        //力控规划
        else
        {
            for (int i = 0; i < 6; i++)
            {
                forceOffsetAvg[i] = forceOffsetSum[i] / 100;
                realForceData[i] = param.fzj_data->at(0).fsr_data[0].fce[i] - forceOffsetAvg[i];
                //若力超过设定的最大值，则只取最大值
                if (std::fabs(realForceData[i]) > forceMax[i])
                {
                    realForceData[i] = realForceData[i] / std::fabs(realForceData[i]) * forceMax[i];
                }
            }
            //转换到机器人身体坐标系
//            double forceSensorMakPe[6]{ 0, 0, 0, 0, -PI/2, 0 };
//            double forceSensorMakPm[16]{ 0 };
//            aris::dynamic::s_pe2pm(forceSensorMakPe,forceSensorMakPm);
            aris::dynamic::s_f2f(*robot.forceSensorMak().prtPm(), realForceData, forceInBody);
            //显示力的初始值
            if (param.count == 100)
            {
                rt_printf("forceOffsetAvg: %f %f %f %f %f %f\n",
                    forceOffsetAvg[0], forceOffsetAvg[1], forceOffsetAvg[2],
                    forceOffsetAvg[3], forceOffsetAvg[4], forceOffsetAvg[5]);
            }
            //人手推动机器人身体移动
            double maxForce{ 0 };
            int index{ 0 };
            for (int i = 0; i < 3; i++)
            {
                if (std::fabs(forceInBody[i]) > maxForce)
                {
                    maxForce = std::fabs(forceInBody[i]);
                    index = i;
                }
            }
            if (maxForce > forceThreshold[index])
            {
                for (int i = 0; i < 3; i++)
                {
                    F[i] = forceInBody[i] / forceMax[i];
                }
            }
            if (std::fabs(forceInBody[4]) > forceThreshold[4])
            {
                F[3] = forceInBody[4] / forceMax[4];
            }

            for (int i = 0; i < 4; i++)
            {
                bodyAcc[i] = (F[i] - C[i] * bodyVel[i] - K[i] * bodyDisp[i]) / M[i];
                bodyVel[i] += bodyAcc[i] * kClockPeriod;
                bodyDisp[i] += bodyVel[i] * kClockPeriod;
            }

            std::copy(bodyDisp, bodyDisp + 4, Peb);
            robot.SetPeb(Peb, beginMak, "231");
            robot.SetPee(Pee, beginMak);

            //for test
            if (param.count % 1000 == 0)
            {
                rt_printf("\nrealForceData: %f %f %f %f %f %f\n",
                    realForceData[0], realForceData[1], realForceData[2],
                    realForceData[3], realForceData[4], realForceData[5]);
                rt_printf("forceInBody: %f %f %f %f %f %f\n",
                    forceInBody[0], forceInBody[1], forceInBody[2],
                    forceInBody[3], forceInBody[4], forceInBody[5]);
                rt_printf("F: %f %f %f %f\n",
                    F[0], F[1], F[2], F[3]);
                rt_printf("bodyAcc: %f %f %f %f\n",
                    bodyAcc[0], bodyAcc[1], bodyAcc[2], bodyAcc[3]);
                rt_printf("bodyVel: %f %f %f %f\n",
                    bodyVel[0], bodyVel[1], bodyVel[2], bodyVel[3]);
                rt_printf("bodyDisp: %f %f %f %f\n\n",
                    bodyDisp[0], bodyDisp[1], bodyDisp[2], bodyDisp[3]);
            }


            //行程检测
            double Pin[18]{ 0 };
            robot.GetPin(Pin);
            bool isOverBound{ false };
            for (int i = 0; i < 18; ++i)
            {
                if (Pin[i] > pinUpBound[i] || Pin[i] < pinLowBound[i])
                {
                    rt_printf("Getting close to the stroke limits.");
                    isOverBound = true;
                }
            }
            if (isOverBound)
            {
                walkBeginCount = param.count + 1;
                isWalking = true;
            }
        }
    }

    if (MbfState::getState().isStopping() && (!isWalking))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
